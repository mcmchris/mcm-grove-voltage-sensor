/*
  MCMVoltSense.cpp - Library for Grove AC Voltage Sensor
  Author: Christopher Mendez, November 3 2022
*/

#include "MCMVoltSense.h"

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

//----------------------------------------------------------------------------------
// Sets the analog pin, calibration factors for the voltage and phase to be used
//----------------------------------------------------------------------------------
void MCMmeter::VoltageStp(unsigned int _analogVin, double _VoltCal, double _PhaseCal)
{
  offsetV = ADC_COUNTS >> 1;
  PhaseCal = _PhaseCal;
  VoltCal = _VoltCal;
  analogVin = _analogVin;
}

//-------------------------------------------------------------------------------------------------
//  Voltage calculation from a window sample of the analog input from the Grove AC Voltage Sensor
//-------------------------------------------------------------------------------------------------
void MCMmeter::analogVoltage(unsigned int cycles, unsigned int timeout)
{

  cycles = cycles/2; // Converting cycles to zero crossings

  int SupplyVoltage = boardVcc();

  unsigned int crossCount = 0;      // Used to measure number of times threshold is crossed.
  unsigned int numberOfSamples = 0; // This is now incremented

  //-------------------------------------------------------------------------------------------------------------------------
  // 1) Waits for the waveform to be close to 'zero' (mid-scale adc) part in sin curve.
  //-------------------------------------------------------------------------------------------------------------------------
  unsigned long start = millis(); // timer for the timeout.

  while (1) // wait for the sine signal to zero cross, break if timeout
  {
    startV = analogRead(analogVin); // using the voltage waveform
    if ((startV < (ADC_COUNTS * 0.51)) && (startV > (ADC_COUNTS * 0.49)))
      break; // check its within range to start from here
    if ((millis() - start) > timeout)
      break;
  }

  //-------------------------------------------------------------------------------------------------------------------------
  // 2) Voltage measurement loop
  //-------------------------------------------------------------------------------------------------------------------------
  start = millis();

  while ((crossCount < cycles) && ((millis() - start) < timeout))
  {
    numberOfSamples++;         // Count number of times looped.
    lastFilteredV = filteredV; // Used for delay/phase compensation

    //-----------------------------------------------------------------------------
    // A) Read in raw voltage and current samples
    //-----------------------------------------------------------------------------
    sampleV = analogRead(analogVin); // Read in raw voltage signal

    //-----------------------------------------------------------------------------
    // B) Apply digital low pass filters to extract the 2.5 V or 1.65 V dc offset,
    //     then subtract this - signal is now centred on 0 counts.
    //-----------------------------------------------------------------------------
    offsetV = offsetV + ((sampleV - offsetV) / ADC_COUNTS);
    filteredV = sampleV - offsetV;

    //-----------------------------------------------------------------------------
    // C) Root-mean-square method voltage
    //-----------------------------------------------------------------------------
    sqV = filteredV * filteredV; // 1) square voltage values
    sumV += sqV;                 // 2) sum

    //-----------------------------------------------------------------------------
    // E) Phase calibration
    //-----------------------------------------------------------------------------
    phaseShiftedV = lastFilteredV + PhaseCal * (filteredV - lastFilteredV);

    //-----------------------------------------------------------------------------
    // G) Find the number of times the voltage has crossed the initial voltage
    //    - every 2 crosses we will have sampled 1 wavelength
    //    - so this method allows us to sample an integer number of half wavelengths which increases accuracy
    //-----------------------------------------------------------------------------
    lastVCross = checkVCross;
    if (sampleV > startV)
      checkVCross = true;
    else
      checkVCross = false;
    if (numberOfSamples == 1)
      lastVCross = checkVCross;

    if (lastVCross != checkVCross)
      crossCount++;
  }

  //-------------------------------------------------------------------------------------------------------------------------
  // 3) Post loop calculations
  //-------------------------------------------------------------------------------------------------------------------------
  // Calculation of the root of the mean of the voltage and current squared (rms)
  // Calibration coefficients applied.

  double V_RATIO = VoltCal * ((SupplyVoltage / 1000.0) / (ADC_COUNTS));
  Vrms = V_RATIO * sqrt(sumV / numberOfSamples);

  // Reset accumulators
  sumV = 0;
  //--------------------------------------------------------------------------------------
}

//-------------------------------------------------------------------------------------------------------------------------
//  Function that measures the supply voltage of the boards.
//-------------------------------------------------------------------------------------------------------------------------
long MCMmeter::boardVcc()
{

  long result;

#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328__) || defined(__AVR_ATmega328P__)
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#elif defined(__AVR_ATmega644__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__)
  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#elif defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_AT90USB1286__)
  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  ADCSRB &= ~_BV(MUX5);                         // Without this the function always returns -1 on the ATmega2560
#elif defined(__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
  ADMUX = _BV(MUX5) | _BV(MUX0);
#elif defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
  ADMUX = _BV(MUX3) | _BV(MUX2);
#endif

#if defined(__AVR__)
  delay(2);                                     // Wait for the reference voltage to stabilize
  ADCSRA |= _BV(ADSC);                          // Convert
  while (bit_is_set(ADCSRA, ADSC))
    ;
  result = ADCL;
  result |= ADCH << 8;
  result = READVCC_CALIBRATION_CONST / result;  // 1100mV*1024 ADC steps
  return result;
#elif defined(__arm__)
  return (3300);                                // Arduino Due
#else
  return (3300);                                // Assuming other architectures works with 3.3V!
#endif
}
