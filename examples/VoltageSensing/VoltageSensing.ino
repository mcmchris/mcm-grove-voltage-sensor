
#include "MCMVoltSense.h"             // Include MCM Volt Sense Library

MCMmeter meter;                       // Create an instance

void setup() {

  Serial.begin(115200);
  
  meter.VoltageStp(A1, 523.56, 1.7);  // Voltage: input pin, calibration, phase_shift

}

void loop() {

  meter.analogVoltage(40,2000);  // Measure the AC voltage. Arguments = (# of AC cycles, timeout)

  float Vrms = meter.Vrms;       // Save the RMS Voltage into a variable.

  Serial.print("Voltage: ");
  Serial.print(Vrms,2);
  Serial.println(" V");

  delay(2000);

}
