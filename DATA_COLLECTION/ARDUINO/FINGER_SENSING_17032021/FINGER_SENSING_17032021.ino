
/**
 * 
 MIT LICENCE:
 Copyright (c) 2022 Øystein Bjelland NTNU
 Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
 The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

 PROJECT DESCRIPTION: Code for reading finger force data using FSR-sensor, and corresponding angular position using potmeter, for arthroscopic puncher.
 Sketch based on example from: learn.adafruit.com/force-sensitive-resistor-fsr/using-an-fsr 
*/

// --------------------------------

// Define FSR-variables

int fsrPin = 0;    // FSR and 10kohm pulldown are connected to a0.
int fsrReading;   // The analog reading from the FSR resistor divider.
//int fsrVoltage;   // The analog reading converted to voltage.
//double fsrResistance;  //  The voltage converted to resistance, can be very big so make "long".
//double fsrConductance; //
double fsrForce;    // Resistance converted to force


// Define potensiometer variables
int potPin = 1;    // Read potmeter from a1
double potVal = 0.00;   
double maxpotVal = 0.00;
double minpotVal = 0.00;
double angle = 0.00;

// Time stamp in milliseconds
double currentMillis = 0;


// --------------------------

void setup() {
  // Begin serial communication at baud rate of 9600;
  Serial.begin(9600);

  delay(5000);

  Serial.println("Please move the lever to the outermost position and hold for 5 seconds");
      while (millis() < 10000) {
        minpotVal = analogRead(potPin);
      }

  delay(5000);

  Serial.println("Please move the lever to the innermost position and hold for 5 seconds");
      while (millis() < 20000) {
        maxpotVal = analogRead(potPin);
      }

  Serial.println("Potentiometer calibration completed");


  delay(5000);
  
  Serial.println("FSR reading, FSR Force [N], angle [deg], time [ms]");  
}



// -------------------------------



void loop() {
 
  // FSR-section
  fsrReading = analogRead(fsrPin);    //Read analog value from fsrPin
  fsrForce = calculateFsrForce(fsrReading);   //Convert analog reading to force [g]

  // Potmeter section
  potVal = analogRead(potPin);
  angle = map(potVal, minpotVal, maxpotVal, 0.00, 1000); // angle in degrees
  double anglePres = angle/100;


  // Print reading to serial monitor
  Serial.print(fsrReading);
  Serial.print("\t");
  Serial.print("");

  Serial.print(fsrForce);
  Serial.print("\t");
  Serial.print("");

  //Serial.print(potVal);
  Serial.print(anglePres);
  Serial.print("\t");
  Serial.print("");

  currentMillis = millis();
  Serial.print(currentMillis);
  Serial.print("\t");
  Serial.println("");

  

}


//---------------------------------------------------------------------------------
// Function to calculate weight from FSR-sensor based on previous calibration data
//---------------------------------------------------------------------------------

double calculateFsrForce(int fsrReading){
  double fsrForce;
  
  fsrForce = 3*pow(10,-7)*pow(fsrReading,3) - 0.0003*pow(fsrReading,2) + 0.7261*fsrReading;
  return fsrForce;
}
