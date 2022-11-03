/*
  MCMVoltSense.cpp - Library for Grove AC Voltage Sensor
  Author: Christopher Mendez, November 3 2022
*/

#ifndef MCMVoltSense_h
#define MCMVoltSense_h

#if defined(ARDUINO) && ARDUINO >= 100

#include "Arduino.h"

#else

#include "WProgram.h"

#endif

// define theoretical vref calibration constant for use in boardVcc()
// 1100mV*1024 ADC steps 
// override in your code with value for your specific AVR chip

#ifndef READVCC_CALIBRATION_CONST
#define READVCC_CALIBRATION_CONST 1126400L
#endif

// to enable 12-bit ADC resolution on Arduino Due,
// include the following line in main sketch inside setup() function:
// analogReadResolution(ADC_BITS);
// otherwise will default to 10 bits, as in regular Arduino-based boards.

#if defined(__arm__)
#define ADC_BITS    12
#else
#define ADC_BITS    10
#endif

#define ADC_COUNTS  (1<<ADC_BITS)


class MCMmeter
{
  public:

    void VoltageStp(unsigned int _analogVin, double _VoltCal, double _PhaseCal);

    void analogVoltage(unsigned int cycles, unsigned int timeout);

    long boardVcc();
    //Useful value variables
    double Vrms;

  private:

    //Set Voltage and current input pins
    unsigned int analogVin;

    //Calibration coefficients
    //These need to be set in order to obtain accurate results
    double VoltCal;
    double PhaseCal;

    //--------------------------------------------------------------------------------------
    // Variable declaration
    //--------------------------------------------------------------------------------------
    int sampleV;                        //sample holds the raw analog read value

    double lastFilteredV,filteredV;          //Filtered is the raw analog value minus the DC offset
    double offsetV;                          //Low-pass filter output


    double phaseShiftedV;                             //Holds the calibrated phase shifted voltage.

    double sqV,sumV;                                  //sq = squared, sum = Sum

    int startV;                                       //Instantaneous voltage at start of sample window.

    boolean lastVCross, checkVCross;                  //Used to measure number of times threshold is crossed.


};

#endif
