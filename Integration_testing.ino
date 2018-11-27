/******************************************************************************
 *  ivWatch Sensor Test Bench System Code
 *  Created: 3/31/2018
 *  Purpose
 *
 *
 *
 *
 *Hardware setup:
 * Arduino Mega ----------------- Model 400
 *    18(TX1)   -----------------    RED  
 *    19(RX1)   -----------------    BROWN
 *    
 * Arduino Mega ----------------- Stepper Motor/BED
 *    02        -----------------    step
 *    03        -----------------    dir
 *    04        -----------------    MS1
 *    05        -----------------    MS2
 *    06        -----------------    MS3
 *    07        -----------------    EN
 *    GND       -----------------    GND
 * 
 * Arduino Mega ----------------- Tension Sensor Amplifier
 *    GND       -----------------    GDN
 *    A10       -----------------    OUT
 * 
 * Arduino Mega ----------------- MPU9250 (Accelerometer, Gyro, Magnetometer)
 *    3.3V      -----------------    VDD
 *    SDA       -----------------    SDA
 *    SCL       -----------------    SCL
 *    GND       -----------------    GND
 *     
 * Arduino Mega ----------------- LCD Shield
 *    GDN       -----------------    VSS
 *    5v        -----------------    VDD
 *    5v/ground -------POT-------    V0
 *    pin 24    -----------------    RS
 *    GND       -----------------    R/W
 *    pin 08    -----------------    EN
 *    pin 09    -----------------    D4
 *    pin 23    -----------------    D5
 *    pin 10    -----------------    D6
 *    pin 22    -----------------    D7
 *    5v        -----10kOhm------    A
 *    GND       -----------------    K
 *
******************************************************************************/

//__________________________________________________________________________
//
//        @@@@@@@@@@@@@@@@@@@@ INCLUDES @@@@@@@@@@@@@@@@@@@@
//__________________________________________________________________________
//
#include "quaternionFilters_modified.h"
#include "MPU9250_modified.h"
#include <LiquidCrystal.h> 




//__________________________________________________________________________
//
//        @@@@@@@@@@@@@@@@@@@@ PINOUT @@@@@@@@@@@@@@@@@@@@
//__________________________________________________________________________
//

//
// Stepper Motor
//
const unsigned int stepperMotor_step = 2;   
const unsigned int stepperMotor_direction = 3;
const unsigned int stepperMotor_MS1 = 4;
const unsigned int stepperMotor_MS2 = 5;
const unsigned int stepperMotor_MS3 = 6;
const unsigned int stepperMotor_enable = 7;

//
// Tension Sensor
//
const unsigned int tensionSensorPin = 10;   // (ANALOG IN)

//
// MPU9250
//
const unsigned int intPin = 12;   
const unsigned int myLed = 13;    

//
// LCD Screen
//
const unsigned int rs = 24;
const unsigned int en = 8;
const unsigned int d4 = 9;
const unsigned int d5 = 23;
const unsigned int d6 = 10;
const unsigned int d7 = 22;



//__________________________________________________________________________
//
//        @@@@@@@@@@@@@@@@@@@@ SYSTEM VARIABLES @@@@@@@@@@@@@@@@@@@@
//__________________________________________________________________________
//

//
// timeing variables
//
unsigned long timeCheck = millis();   // elapsed time (ms)
unsigned long loopStartTime = 0;    // time at beginning of the loop()
unsigned long dataTimeStamp = 0;    // time when data packet is sent to MATLAB
int timeStep = 2000;   // interval in which to actuate stepper motor (ms)
int orientationTimeStep = 10000;    // 10 s for orientation to run

//
// sensor/actuator variables
//
int stepperMotor_x;   // count variable 
float tensionData;    // amount of force being applied
float tensionVoltage;   // tenion Voltage  
float tensionLimit = 1.8; // tension Limit
float tensionScaleFactor = 2.2f/tensionLimit;
float model400_in = 0;    // data read in from Model 400

//
// flag variables
//
bool setupFlag = false;    // check to see when to move to stage 2
int startCheck = 0;   // data from Matlab to indicate starting of the system
int stopCheck = 0;    // flag to alert System Stop request
bool stopFlag = false;
bool tensionFlag = false;    // flag to indicate if tension has exceded max


//
// LCD Discplay setup
//
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);    

MPU9250 myIMU;    // create an instance of the MPU9250 class



//__________________________________________________________________________
//
//        @@@@@@@@@@@@@@@@@@@@ SETUP FNCTION @@@@@@@@@@@@@@@@@@@@
//__________________________________________________________________________
//
void setup() {
  // Serial Setup for comms with MATLAB
  Serial.begin(19200);    
  // Serial Setup for comms with Model 400
  Serial1.begin(19200);   
  // Wire setup for comms with MPU9250
  Wire.begin();       
      
  //
  //########## Stepper Motor Pin Setup ##########
  //
  pinMode(stepperMotor_step, OUTPUT);
  pinMode(stepperMotor_direction, OUTPUT);
  pinMode(stepperMotor_MS1, OUTPUT);
  pinMode(stepperMotor_MS2, OUTPUT);
  pinMode(stepperMotor_MS3, OUTPUT);
  pinMode(stepperMotor_enable, OUTPUT);

  //
  // Big Easy Driver Setup (step, direction, microstep, and enable pins
  //
  BEDSetup(); 

  lcd.begin(16, 4);   // set up the LCD's number of columns(16) and rows(4):
  lcd.print("Orientation");



  //__________________________________________________________________________
  //
  //     @@@@@@@@@@@@@@@@@@@@ MPU9250 SETUP AND READ @@@@@@@@@@@@@@@@@@@@
  //__________________________________________________________________________
  //

  /*###############################################################################################################*/
  
  //
  // Set up MPU9250 the interrupt pin
  //
  pinMode(intPin, INPUT);
  digitalWrite(intPin, LOW);
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, HIGH);

  // Read the WHO_AM_I register, this is a good test of comms with MPU9250
  byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);

  if (c == 0x71) // WHO_AM_I should always be 0x68
  {
    // Start by performing self test and reporting values
    myIMU.MPU9250SelfTest(myIMU.SelfTest);

    // Calibrate gyro and accelerometers, load biases in bias registers
    myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);
    myIMU.initMPU9250();
    
    // Read the WHO_AM_I register of the magnetometer, this is a good test of comms with AK8963
    byte d = myIMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
    
    // Get magnetometer calibration from AK8963 ROM
    myIMU.initAK8963(myIMU.magCalibration);
  } // if (c == 0x71)

  //
  // Condition if IMU is not detected
  // 
  else
  {
    Serial.print("Could not connect to MPU9250: 0x");
    Serial.println(c, HEX);
    while(1) ; // Loop forever if communication doesn't happen
  }

  //
  // Read in MPU9250 data
  //
  while (setupFlag == false)
  {
    // If intPin goes high, all data registers have new data
    // On interrupt, check if data ready interrupt
    if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
    {  
      myIMU.readAccelData(myIMU.accelCount);  // Read the x/y/z adc values
      myIMU.getAres();
  
      // Now we'll calculate the accleration value into actual g's
      // This depends on scale being set
      myIMU.ax = (float)myIMU.accelCount[0]*myIMU.aRes; // - accelBias[0];
      myIMU.ay = (float)myIMU.accelCount[1]*myIMU.aRes; // - accelBias[1];
      myIMU.az = (float)myIMU.accelCount[2]*myIMU.aRes; // - accelBias[2];
  
      myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values
      myIMU.getGres();
  
      // Calculate the gyro value into actual degrees per second
      // This depends on scale being set
      myIMU.gx = (float)myIMU.gyroCount[0]*myIMU.gRes;
      myIMU.gy = (float)myIMU.gyroCount[1]*myIMU.gRes;
      myIMU.gz = (float)myIMU.gyroCount[2]*myIMU.gRes;
  
      myIMU.readMagData(myIMU.magCount);  // Read the x/y/z adc values
      myIMU.getMres();
      // User environmental x-axis correction in milliGauss, should be
      // automatically calculated
      myIMU.magbias[0] = +470.;
      // User environmental x-axis correction in milliGauss TODO axis??
      myIMU.magbias[1] = +120.;
      // User environmental x-axis correction in milliGauss
      myIMU.magbias[2] = +125.;
  
      // Calculate the magnetometer values in milliGauss
      // Include factory calibration per data sheet and user environmental
      // corrections
      // Get actual magnetometer value, this depends on scale being set
      myIMU.mx = (float)myIMU.magCount[0]*myIMU.mRes*myIMU.magCalibration[0] -
                 myIMU.magbias[0];
      myIMU.my = (float)myIMU.magCount[1]*myIMU.mRes*myIMU.magCalibration[1] -
                 myIMU.magbias[1];
      myIMU.mz = (float)myIMU.magCount[2]*myIMU.mRes*myIMU.magCalibration[2] -
                 myIMU.magbias[2];
    } 

    /*###############################################################################################################*/
      
    // Must be called before updating quaternions!
    myIMU.updateTime();

    //
    // Call the Quaternion Calculations
    //
    MahonyQuaternionUpdate(myIMU.ax, myIMU.ay, myIMU.az, myIMU.gx*DEG_TO_RAD,
                           myIMU.gy*DEG_TO_RAD, myIMU.gz*DEG_TO_RAD, myIMU.my,
                           myIMU.mx, myIMU.mz, myIMU.deltat);

    //
    // Serial print and/or display at 0.5 s rate independent of data rates
    //
    myIMU.delt_t = millis() - myIMU.count;
    
    if (myIMU.delt_t > 500)
    {
  
      myIMU.yaw   = atan2(2.0f * (*(getQ()+1) * *(getQ()+2) + *getQ() *
                    *(getQ()+3)), *getQ() * *getQ() + *(getQ()+1) * *(getQ()+1)
                    - *(getQ()+2) * *(getQ()+2) - *(getQ()+3) * *(getQ()+3));
      myIMU.pitch = -asin(2.0f * (*(getQ()+1) * *(getQ()+3) - *getQ() *
                    *(getQ()+2)));
      myIMU.roll  = atan2(2.0f * (*getQ() * *(getQ()+1) + *(getQ()+2) *
                    *(getQ()+3)), *getQ() * *getQ() - *(getQ()+1) * *(getQ()+1)
                    - *(getQ()+2) * *(getQ()+2) + *(getQ()+3) * *(getQ()+3));
                    
      myIMU.pitch *= RAD_TO_DEG;
      myIMU.yaw   *= RAD_TO_DEG;
      
      // Declination of SparkFun Electronics (40°05'26.6"N 105°11'05.9"W) is
      //   8° 30' E  ± 0° 21' (or 8.5°) on 2016-07-19
      // - http://www.ngdc.noaa.gov/geomag-web/#declination
      myIMU.yaw   -= 8.5;
      myIMU.roll  *= RAD_TO_DEG;
      

      //
      // Print Roll Pitch & Yaw to the LCD screen
      //
      lcd.setCursor(0, 1);
      lcd.print("Roll: " + String(myIMU.roll));
      lcd.setCursor(4, 2);
      lcd.print("Pitch: " + String(myIMU.pitch));
      lcd.setCursor(4,3);
      lcd.print("Yaw: " + String(myIMU.yaw));

      //
      // Send Orientation data in one line to matlab at time step 0.5s
      //
      Serial.println("Orientation Data    " + String(myIMU.roll) + "    " + String(myIMU.pitch) + "    " + String(myIMU.yaw) + "\n"); 
        
      myIMU.count = millis();
      myIMU.sumCount = 0;
      myIMU.sum = 0;
    } // if (myIMU.delt_t > 500)


    //
    // Continuously read serial data from MATLAB
    //
    if (Serial.available())
    {
      startCheck = Serial.read();
      Serial.println(startCheck);
    }

    //
    // If startCheck = 49 begin test
    //
    if(startCheck == 49)
    {
      setupFlag = true;
    }
    
  } // while (millis() - timeCheck) < orientationTimeStep

  //
  // Main Loop Start time set
  // Time Check set
  //
  loopStartTime = millis();
  timeCheck = millis();

}



//__________________________________________________________________________
//
//        @@@@@@@@@@@@@@@@@@@@ LOOP/MAIN FNCTION @@@@@@@@@@@@@@@@@@@@
//__________________________________________________________________________
//
void loop() {

  //
  // Read in Tension Data 
  // Convert to voltage
  //
  tensionData = analogRead(tensionSensorPin);
  tensionVoltage = (tensionData *5)/ 1023;

  //
  // Read in Stop Flag from MATLAB
  //
  if (Serial.available())
  {
    stopCheck = Serial.read();
    if (stopCheck == 49)
    {
      stopFlag = true;
    }
  }
  
  //
  // Time stamp data before sending to Matlab
  //
  dataTimeStamp = millis() - loopStartTime;

  //
  // Read data on Serial Line 1 (MODEL400)
  //
  model400_in = Serial1.read();

  //
  // Write data to Serial Line 0 (MATLAB)
  //
  Serial.println(String(dataTimeStamp) + "    " + String(tensionVoltage*tensionScaleFactor) + "    " + String(model400_in) + "    " + String(tensionFlag));


  //
  // Ensure tension stays under 2.2 Volts
  //
  if (tensionVoltage >= tensionLimit)
  {
    tensionFlag = true;
  }


  //
  // Run Branch every (timeStep)(ms)
  //
  if ((millis() - timeCheck) > timeStep)
  {
    
    if (!tensionFlag & !stopFlag)
    {
      // Pull enable pin low to set FETs active and allow motor control
      digitalWrite(stepperMotor_enable, LOW);

      // rotate stepper motor
      StepMotor();    
    }

    timeCheck = millis();   // reset time Check
    
  }

  //
  // Ensure code runs at 100+ ms intervals
  //
  delay(100);
}



//__________________________________________________________________________
//
//        @@@@@@@@@@@@@@@@@@@@ SYSTEM FNCTIONS @@@@@@@@@@@@@@@@@@@@
//__________________________________________________________________________
//

//
//Reset Big Easy Driver pins to default states
//
void BEDSetup()
{
  digitalWrite(stepperMotor_step, LOW);
  digitalWrite(stepperMotor_direction, HIGH);   // Pull HIGH to step clockwise
  digitalWrite(stepperMotor_MS1, HIGH);   // \\
  digitalWrite(stepperMotor_MS2, HIGH);   //  }} Pull all 3 HIGH for 16th step
  digitalWrite(stepperMotor_MS3, HIGH);   // //
  digitalWrite(stepperMotor_enable, HIGH);
}


//
// Turn Motor   This function seems to take about 2 seconds to complete as is
//
void StepMotor()
{
  for(stepperMotor_x= 1; stepperMotor_x<10; stepperMotor_x++)  //Loop the forward stepping enough times for motion to be visible
  {
    digitalWrite(stepperMotor_step,HIGH); //Trigger one step forward
    delay(1);
    digitalWrite(stepperMotor_step,LOW); //Pull step pin low so it can be triggered again
    delay(1);
  }
}
