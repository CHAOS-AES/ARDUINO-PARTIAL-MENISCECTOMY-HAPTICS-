
// Define FSR-pin
#define fsrpin A0

int potPin = 1;
int potVal = 0;
int maxpotVal = 0;
int minpotVal = 0;
int angle = 0;

// Define variable to store sensor reading:
int fsrreading;
//int fsrForce = 0;

// Time stamp in mnilliseconds
unsigned long currentMillis = 0;

void setup() {
  // Begin serial communication at baud rate of 9600;
  Serial.begin(9600);
  
  Serial.println("Please move the lever to the outermost position and hold for 5 seconds");
      while (millis() < 7000) {
        minpotVal = analogRead(potPin);
      }

  Serial.println("Please move the lever to the innermost position and hold for 5 seconds");
      while (millis() < 14000) {
        maxpotVal = analogRead(potPin);
      }

  Serial.println("Potentiometer calibration completed");



  
  Serial.println("FSR reading, angle [deg], time [ms]");  
}



void loop() {
 
  // Read FSR-pin and store the output as fsrreading
  fsrreading = analogRead(fsrpin);
  //fsrForce = map(fsrreading, 0, 640, 0, 1000);

  // Read potmeter
  potVal = analogRead(potPin);
  angle = map(potVal, minpotVal, maxpotVal, 0, 10); // angle in degrees
  //angle = map(potVal, 0, 1023, 0, 360); // angle in degrees

  // Print reading to serial monitor
  Serial.print(fsrreading);
  //Serial.print(fsrForce);
  Serial.print("\t");
  Serial.print("");

  Serial.print(angle);
  Serial.print("\t");
  Serial.print("");

  currentMillis = millis();
  Serial.print(currentMillis);
  Serial.print("\t");
  Serial.println("");

  delay(5); // Delay 5ms.

}
