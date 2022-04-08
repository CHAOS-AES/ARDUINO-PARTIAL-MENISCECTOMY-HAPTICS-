
/**

 MIT LICENCE:
 Copyright (c) 2022 Øystein Bjelland NTNU
 Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
 The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

 
 PROJECT DESCRIPTION: Real time finite-element based haptic rendering with Kalman Filter estimation of force from empirical data. Interfacing with ARDUINO DUE microcontroller, BGM2804 BLDC Motor, CUI AMT 102V Encoder.
 Based on SimpleFOC library for BLDC control (Copyright (c) 2020 Antun Skuric) 
 Finite Element implementation based on MATLAB code by Morten D. Pedersen, NTNU

  */
  
//*************************************************************************************
//DECLARE DEPENDENCIES START***********************************************************
 
#include <SimpleFOC.h> // FOR CONTROLLING BLDC Motor: https://github.com/simplefoc/Arduino-FOC
#include <hapticFEM.hpp>
#include <BasicLinearAlgebra.h> // FOR MATRIX OPERATIONS: https://github.com/tomstewart89/BasicLinearAlgebra 
#include <ElementStorage.h> // For BLA library

//using namespace BLA; //Using namespace basic linear algebra

//DECLARE DEPENDENCIES END************************************************************
//*************************************************************************************


//*************************************************************************************
//INITIALIZATION START*****************************************************************

  #define DOF 5 // Defines number of elements in rubber band model
  

  //Define Kalman Filter matrices
  BLA::Matrix<2,2> Q = {7.0275, 0.02772, 0.2772, 6.1024}; //System covariance matrix
  BLA::Matrix<1> R_FEM = {0.001}; // Measurement variance from FEM
  BLA::Matrix<1> R_empirical = {7.742}; // Measurement variance from empirical measurements
  BLA::Matrix<1> R = R_FEM;

  BLA::Matrix<2,2> A = {1, 1, 0, 1}; //System matrix
  BLA::Matrix<2,2> Ad = A;
  BLA::Matrix<2,1> B = {1, 0};       //Input matrix
  BLA::Matrix<2,1> Bd = B;
  BLA::Matrix<1,2> H = {1, 0};       //Measurement matrix
  BLA::Matrix<2,1> HTransposed = ~H; //Measurement matrix transposed
  BLA::Matrix<1> u = {0};            //Input     
  BLA::Matrix<2,2> Id = {1, 0, 0, 1};//Identity matrix  

  BLA::Matrix<1> forceMeasurement = {0};
  double FEMforceMeasurement = 0;
  float maxFemForce = 0;
  float yieldForce = 7; // [N] Max FEM yield force
  float dTheta = 0;
  float xhDegPreviousStep = 0;
  BLA::Matrix<2,2> Pplus = {0, 0, 0, 0};

  BLA::Matrix<2,1> xhat = {0, 0}; // Initial conditions. Zero force, zero stiffness.
  
  


//*************************************************************************************
//FUNCTION PERFORMING EMPIRICAL FORCE CALCULATION START********************************

BLA::Matrix<1> empiricalData(float xhDeg){
  //This function takes in a position value [deg] and returns a force signal [N]

  //BLA::Matrix<1> empForceMeasurement;
  double empForceMeasurement;
  double theta = (xhDeg - 16)/2; //Offset and scaling of theta range to fit FEM. Remove if not needed.
  
  // <row, column>
  BLA::Matrix<76,1> empForce;
  empForce = {0.000,0.000,0.000,0.008,0.008,0.000,0.000,0.000,0.000,0.008,0.008,0.008,0.008,0.008,0.008,0.008,0.000,0.084,0.228,0.823,1.006,1.789,2.217,2.791,3.313,3.798,4.096,4.187,4.220,4.302,4.344,4.220,4.327,4.236,4.369,4.253,4.277,4.277,4.145,4.072,3.894,3.894,3.999,4.154,3.966,4.015,3.798,3.726,3.750,3.774,3.814,3.679,3.806,3.894,3.950,4.056,4.039,3.983,4.007,3.910,3.894,3.886,4.015,4.031,3.958,3.950,4.031,4.007,4.064,4.015,4.113,4.031,3.983,4.056,4.080,4.344};
  
  BLA::Matrix<76,1> empTheta;
  empTheta = {2.079,2.188,2.300,2.414,2.526,2.636,2.742,2.842,2.939,3.033,3.127,3.222,3.322,3.430,3.547,3.675,3.813,3.961,4.118,4.283,4.453,4.629,4.807,4.988,5.171,5.357,5.545,5.737,5.932,6.132,6.335,6.542,6.752,6.966,7.183,7.404,7.627,7.854,8.083,8.315,8.546,8.777,9.003,9.221,9.429,9.621,9.796,9.950,10.082,10.192,10.280,10.349,10.400,10.438,10.465,10.485,10.500,10.514,10.528,10.543,10.559,10.576,10.595,10.615,10.635,10.656,10.675,10.694,10.712,10.729,10.745,10.761,10.776,10.791,10.806,10.820};

  // Linear interpolation

  for (int i = 0; i<75; i++){
      if ((theta > empTheta(i)) && (theta <= empTheta(i+1))){
          empForceMeasurement = empForce(i) + ((empForce(i+1) - empForce(i))*((theta - empTheta(i))/(empTheta(i+1) - empTheta(i))));
      } 
  }

    if (empForceMeasurement > 4.29){
    empForceMeasurement = 4.29;
  } else if (empForceMeasurement < 0.01){
    empForceMeasurement = 0.01;
  }
  
  return empForceMeasurement;

}

//FUNCTION PERFORMING EMPIRICAL FORCE CALCULATION END**********************************
//*************************************************************************************



//*************************************************************************************
//DECLARE MOTOR AND ENCODER START******************************************************

// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(7);
BLDCDriver3PWM driver = BLDCDriver3PWM(9, 5, 6, 8);

// encoder instance
Encoder encoder = Encoder(3, 2, 2048);

// Interrupt routine intialisation
// channel A and B callbacks
void doA(){encoder.handleA();}
void doB(){encoder.handleB();}

float target_voltage = 0; //Voltage set point varable

// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&target_voltage, cmd); }

double xh = 0;       //Handle position [rad]
double vh = 0;       //Handle velocity [rad/s]

unsigned long currentMillis = 0; // [s]
unsigned long previousMillis = 0; // [s]
unsigned long visualRefreshRate = 30; // [Hz] (=30Hz)

RubberBand<DOF> rubber_band;

//DECLARE MOTOR AND ENCODER END********************************************************
//*************************************************************************************





  
void setup() { 

  //*************************************************************************************
  //Initiate BLDC and Encoder start******************************************************
  
  // initialize encoder sensor hardware
  encoder.init();
  encoder.enableInterrupts(doA, doB); 
  // link the motor to the sensor
  motor.linkSensor(&encoder);

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  driver.init();
  // link driver
  motor.linkDriver(&driver);

  // aligning voltage
  motor.voltage_sensor_align = 3.3; //12?
  // choose FOC modulation (optional)
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

  // set motion control loop to be used
  motor.controller = MotionControlType::torque;

  // use monitoring with serial 
  Serial.begin(115200);
  // comment out if not needed
  motor.useMonitoring(Serial);

  // initialize motor
  motor.init();
  // align sensor and start FOC
  motor.initFOC();

  Serial.println(F("Motor ready."));
  _delay(1000);


  //*************************************************************************************
  //Assemble stiffness matrix from mesh size (N)*****************************************
  
  rubber_band.setup();

  
}

  

void loop() {

  //*************************************************************************************
  //Run FOC loop and get position data*************************************************** 
  motor.loopFOC();

  xh = encoder.getAngle();
  float xhDeg = (xh/M_PI)*180;
  vh = encoder.getVelocity();
  

  //*************************************************************************************
  //CALCULATE FEM-FORCE AND DEFORMED SHAPE FROM TOOL POSITION****************************
  
  auto d = rubber_band.loop(xhDeg);

  //Vizualize deformed shape if needed
  //Serial << "deformedShape: " << d.deformedShape << '\n';

  //Use FEM-simulation as force measurement, and clamp signal within range

  if (d.force < 0){
    FEMforceMeasurement = 0;
  }else if ((d.force > yieldForce)){
    FEMforceMeasurement = yieldForce;
  }else {
    FEMforceMeasurement = d.force;
  }

  //*************************************************************************************
  //KALMAN FILTER HAPTIC FORCE ESTIMATION************************************************

  //STEP 0: Position stepping
  dTheta = xhDeg - xhDegPreviousStep;
  xhDegPreviousStep = xhDeg;
  
  Ad(0,1) = dTheta;
  Bd = B; 

  //STEP 1: Use FEM-simulation as force measurement, and assign uncertainty

  forceMeasurement(0) = FEMforceMeasurement;  //Take measurement
  R = R_FEM;      //Assign measurement uncertainty

  //STEP 2: Estimate haptic force from FEM
  //Predict
  xhat = Ad*xhat + B*u; // A priori force estimate
  BLA::Matrix<2,2> AdTransposed = ~Ad;   // Transpose Ad
  BLA::Matrix<2,2> Pmin = Ad*Pplus*AdTransposed + Q; // A priori covariance matrix

  //Correct
  BLA::Matrix<1> HPHR = H*Pmin*HTransposed + R;   //Calulate (H*Pmin*HTransposed + R)
  BLA::Matrix<1> HPHR_Inverse = HPHR;             //Prepare for taking inverse of HPHR
  Invert(HPHR_Inverse);                           //Take inverse of HPHR (COULD BE PROBLEMS BECAUSE MATRIX IS NOT SQUARE)
  BLA::Matrix<2,1> K = Pmin*HTransposed*HPHR_Inverse; //Compute Kalman Gain
  xhat = xhat + K*(forceMeasurement - H*xhat);  // A posteriori force estimate
  Pplus = (Id - K*H)*Pmin; // A posteriori covariance matrix
  
  //STEP 3: Use empirical data as force measurement
  BLA::Matrix<1> empForceMeasurement = empiricalData(xhDeg);  //Take measurement
  forceMeasurement(0) = empForceMeasurement(0);
  R = R_empirical;  //Assign measurement uncertainty

  //STEP 4: Estimate haptic force from empirical data
   //Predict
  xhat = Ad*xhat + B*u; // A priori force estimate
  AdTransposed = ~Ad;   // Transpose Ad
  Pmin = Ad*Pplus*AdTransposed + Q; // A priori covariance matrix

  //Correct
  HPHR = H*Pmin*HTransposed + R;   //Calulate (H*Pmin*HTransposed + R)
  HPHR_Inverse = HPHR;             //Prepare for taking inverse of HPHR
  Invert(HPHR_Inverse);                           //Take inverse of HPHR (COULD BE PROBLEMS BECAUSE MATRIX IS NOT SQUARE)

  K = Pmin*HTransposed*HPHR_Inverse; //Compute Kalman Gain
  xhat = xhat + K*(forceMeasurement - H*xhat);  // A posteriori force estimate
  Pplus = (Id - K*H)*Pmin; // A posteriori covariance matrix

  float forceEstimate = xhat(0);

  //*************************************************************************************
  //CONVERT FORCE SIGNAL TO MOTOR CONTROL SIGNAL (VOLTAGE)*******************************

  // Mapping force to voltage
  //double target_voltage = -d.force*1.5; //CASE 1: Uncomment to use only FEM-signal for haptic signal
  //double target_voltage = -empForceMeasurement(0)*2.7; // CASE 2: Uncomment to use only Empirical signal.
  double target_voltage = -forceEstimate*2.25; //CASE 3: Uncomment to use kalman filter estimation for haptic signal

  // Cap signal to prevent overloading the motor
  if(target_voltage < -12){
      target_voltage = -12;
  } else if (target_voltage > 0){
    target_voltage = 0;
  }

  // Send signal to motor
  motor.move(target_voltage);


  //*************************************************************************************
  //SEND MESH TO SERIAL MONITOR FOR VISUALIZATION****************************************

  //Serial << deformedShape << '\n'; // Printing deformed shape matrix to serial monitor

  // Concatenate length of string according to mesh size
  String mesh(d.deformedShape(0));
  for (int i = 1; i< DOF; i++){
    mesh.concat(","); mesh.concat(d.deformedShape(i)); 
  }
  
  //We only send data to serial monitor at 30 Hz (or each 33th millisecond).
  double elapsedTime = currentMillis - previousMillis;
  
  if(elapsedTime >= ((1/visualRefreshRate)*1000)){
  Serial.println(mesh); //<-----UNCOMMENT TO SEND TO PROCESSING
  //Serial.println(elapsedTime);
  previousMillis = currentMillis; 
  
  }


  //*************************************************************************************
  //VISUALIZE CONTROL DATA (FOR DEBUGGING PURPOSES ONLY)*********************************
  
    
//    Serial.print(-target_voltage);
//    Serial.print("\t");
//    Serial.print(d.force);
//    Serial.print("\t");
//    Serial.print(xhDeg);
//    Serial.print("\t");
//
//    Serial.print(FEMforceMeasurement);
//    Serial.print("\t");
//    Serial.print(empForceMeasurement(0));
//    Serial.print("\t");
//    Serial.print(forceEstimate);
//    Serial.print("\t");
//    Serial.print(vh);
//    Serial.print("\t");   
//    Serial.print(micros());
//    Serial.println("\t");


  
}
