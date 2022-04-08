// Code for reading finger force data using FSR-sensor, and corresponding angular position using potmeter, for arthroscopic grasper.
// By Øystein Bjelland, CPS-lab, Department of ICT and Natural Sciences, NTNU

// Code based on learn.adafruit.com/force-sensitive-resistor-fsr/using-an-fsr 

// --------------------------------

// Define FSR-variables

int fsrPin = 0;    // FSR and 10kohm pulldown are connected to a0.
int fsrReading;   // The analog reading from the FSR resistor divider.
int fsrVoltage;   // The analog reading converted to voltage.
double fsrResistance;  //  The voltage converted to resistance, can be very big so make "long".
double fsrConductance; //
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
  fsrVoltage = map(fsrReading, 0, 1023, 0, 5000); //analog voltage reading ranges from 0 to 1023 which maps to 0V to 5v (=5000mV)

  if (fsrVoltage == 0) {
      // No pressure!
  } else {
      // The voltage = Vcc * R / (R + FSR), where R = 10kOhm and Vcc = 5V
      // So FSR = ((Vcc - V)*R / V
      fsrResistance = 5000 - fsrVoltage;  // milliVolts --> 5V = 5000mV
      fsrResistance *= 10000;           // 10kOhm resistor. (equals fsrResistance = fsrResistance * 10 000)
      fsrResistance /= fsrVoltage;      // fsrResistance = fsrResistance / fsrVoltage

      fsrConductance = 1000000;   // measure in microMhos G = 1/R
      fsrConductance /= fsrResistance;

      // Use two FSR guide graphs to approximate the force
      if (fsrConductance <= 1000) {
        fsrForce = fsrConductance / 80; // fsrForce in Newtons
      } else {
        fsrForce = fsrConductance - 1000;
        fsrForce /= 30;   //fsrForce in Newtons
      }
  }

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

   delay(50); // Delay 5ms. But we dont really want any delay.

}
