
//Cirena Arabit

/*
This program controls a car via bluetooth and contains two autonomous features:
crash avoidance and line tracking

This program will also find the greatest acceleration during a crash avoidance run
and store that value to EEPROM

*/

#include <SoftwareSerial.h> // SoftwareSerial for serial communication with HM10 bluetooth module.
#include <ArduinoBlue.h> // ArduinoBlue bluetooth library
#include <Wire.h>
#include <EEPROM.h>

const unsigned long BAUD_RATE = 9600;

#define enA 11
#define in1 12
#define in2 13

#define enB 10
#define in3 9
#define in4 8

//for controllling car speed
int val1 = 0;
int val2 = 0;
int speed = 0;
int reducedSpeed = 0;

// HM10 BLUETOOTH MODULE PINS
//put tx here in 6
const int BLUETOOTH_TX = 6;
const int BLUETOOTH_RX = 7;
SoftwareSerial softSerial(BLUETOOTH_TX, BLUETOOTH_RX); // Object for serial communication to HM 10 bluetooth module using digital pins.
ArduinoBlue phone(softSerial); // Object for smartphone robot control.


//control the button on ArduinoBlue
int button;
boolean lineEnabled = false;
boolean driveEnabled = true; 
boolean crashEnabled = false; 


//for line tracking
const int pinIRd = 3;
const int pinIRd2 = 5;
int IRvalueD = 0;
int IRvalueD2 = 0;

//for the sonic sensor
int trigPin = A3;
int echoPin = A2;
float speedOfSound = 776.5; // Speed of sound in miles per hour when temp is 77 degrees.
float distanceMeasured;


//for the MPU 6050
long accelX, accelY, accelZ;
float gForceX, gForceY, gForceZ;
long gyroX, gyroY, gyroZ;
float rotX, rotY, rotZ;
float maxGForceX = 0;
float maxGForceY = 0;


void setup() 
{
  Serial.begin(9600);
  Wire.begin();
  softSerial.begin(BAUD_RATE);

  setupMPU();

  //setup motor
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  //set tracking sensor
  pinMode(pinIRd, INPUT);
  pinMode(pinIRd2, INPUT);

  //set up sonic sensor
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

void setupMPU(){
Wire.beginTransmission(0b1101000); //This is the I2C address of the MPU (b1101000/b1101001 for AC0 low/high datasheet sec. 9.2)
Wire.write(0x6B); //Accessing the register 6B - Power Management (Sec.4.28)
Wire.write(0b00000000); //Setting SLEEP register to 0. (Required; see Note on p. 9)
Wire.endTransmission();
Wire.beginTransmission(0b1101000); //I2C address of the MPU
Wire.write(0x1B); //Accessing the register 1B - Gyroscope Configuration (Sec. 4.4)
Wire.write(0x00000000); //Setting the gyro to full scale +/- 250deg./s
Wire.endTransmission();
Wire.beginTransmission(0b1101000); //I2C address of the MPU
Wire.write(0x1C); //Accessing the register 1C - Acccelerometer Configuration (Sec. 4.5)
Wire.write(0b00000000); //Setting the accel to +/- 2g
Wire.endTransmission();
}

void loop() 
{
  button = phone.getButton();

  //Press button--enable line tracking or enable/disable crash avoidance
    if (button != -1)
  {
    //enable line tracking, disable driving
    if(button == 1)
    { 
      driveEnabled = false;
      lineEnabled = true;
    }
    //enable crash avoidance
    else if (button == 2)
    {
      crashEnabled = !crashEnabled;
    }

  }

  //line tracking
    while(lineEnabled == true)
  {
    
    IRvalueD = digitalRead(pinIRd);
    IRvalueD2 = digitalRead(pinIRd2);

    lineTracking();


    //disable line tracking
    button = phone.getButton();
    if(button != -1)
    {

      if(button == 1)
      {
       lineEnabled = false; 
       driveEnabled = true; 
      }
      
    }



  }

  //drive the car
  if(driveEnabled == true) 
  {
    controlDrive();

    //crash avoidance
    if(crashEnabled == true)
    {
      //sonic sensor - get the distance from object
      float distance = measureDistance();
      avoidCrash(distance, maxGForceX, maxGForceY);

      //read the MPU 6050
      recordAccelRegisters();
      recordGyroRegisters();

      //if x accel is greater than previous x accel, store in maxGForceX
      if(gForceX > maxGForceX)
      {
      maxGForceX = gForceX;
      Serial.print("NEW MAX XGFORCE: "); Serial.println(maxGForceX);
      }
      //if y accel is greater than previous y accel, store in maxGForceY
      if(gForceY > maxGForceY)
      {
      maxGForceY = gForceY;
      Serial.print("NEW MAX YGFORCE: "); Serial.println(maxGForceY);
      }

    }

  }



}

void recordAccelRegisters()
{
Wire.beginTransmission(0b1101000); //I2C address of the MPU
Wire.write(0x3B); //Starting register for Accel Readings
Wire.endTransmission();
Wire.requestFrom(0b1101000,6); //Request Accel Registers (3B - 40)
while(Wire.available() < 6);
accelX = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
accelY = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
accelZ = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ
processAccelData();
}

void processAccelData()
{ 
gForceX = accelX / 16384.0;
gForceY = accelY / 16384.0;
gForceZ = accelZ / 16384.0;
}

void recordGyroRegisters()
{ 
Wire.beginTransmission(0b1101000); //I2C address of the MPU
Wire.write(0x43); //Starting register for Gyro Readings
Wire.endTransmission();
Wire.requestFrom(0b1101000,6); //Request Gyro Registers (43 - 48)
while(Wire.available() < 6);
gyroX = Wire.read()<<8|Wire.read(); //Store first two bytes into gyroX
gyroY = Wire.read()<<8|Wire.read(); //Store middle two bytes into gyroY
gyroZ = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ
processGyroData();
}


void processGyroData()
{ 
rotX = gyroX / 131.0;
rotY = gyroY / 131.0;
rotZ = gyroZ / 131.0;
}


// Configures the motor controller to stop the motors.
void motorBrake() {
  digitalWrite(enA, LOW);
  digitalWrite(enB, LOW);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  digitalWrite(enA, HIGH);
  digitalWrite(enB, HIGH);
}

// Configures the motor controller to have the robot move forward.
void motorSetForward() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

// Configures the motor controller to have the robot move backwards.
void motorSetBackward() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

//For the sonic sensor
//Measures distance
float measureDistance()
{
 digitalWrite(trigPin, LOW); // Set trigger pin low
 delayMicroseconds(2000); // Let signal settle
 digitalWrite(trigPin, HIGH); // Set trigPin high
 delayMicroseconds(15); // Delay in high state
 digitalWrite(trigPin, LOW); // Ping has now been sent
 delayMicroseconds(10); // Delay in low state


 float pingTime = pulseIn(echoPin, HIGH); // pingTime is presented in microseconds
 pingTime /= 1000000.0; // convert pingTime to seconds
 pingTime /= 3600.0; // convert pingtime to hours


 float targetDistance = speedOfSound * pingTime / 2; // This will be in miles
 targetDistance *= 63360; // Convert miles to inches
 return targetDistance;
}

void storeAccelX(float MAXgForceX)
{
  float storedGForceX = EEPROM.put(0,MAXgForceX);
  Serial.print("X Accel Stored: "); Serial.print(storedGForceX);
  delay(5);
}

void storeAccelY(float MAXgForceY)
{
  float storedGForceY = EEPROM.put(4,MAXgForceY);
  Serial.print("\t Y Accel Stored: "); Serial.println(storedGForceY);
  delay(5);
}


//Sonic sensor; detect distance from object--stop if too close
//store max acceleration in X and Y direction when stopped
void avoidCrash(float distance, float MAXgForceX, float MAXgForceY)
{

  if(distance < 10)
  {
    Serial.println("TOO CLOSE!");

    storeAccelX(MAXgForceX);
    storeAccelY(MAXgForceY);

    driveEnabled = false;

    motorSetBackward();
    
    analogWrite(enA, 90);
    analogWrite(enB, 90);
    delay(200);

    motorBrake();
   
    driveEnabled = true;
  }


}

void lineTracking()
{
 
  //FOR LINE TRACKING
  //right sensor IRvalueD, left sensor IRvalueD2
  IRvalueD = digitalRead(pinIRd);
  IRvalueD2 = digitalRead(pinIRd2);

  //both sensors detect black -- move forward
  if ((IRvalueD == 1 && IRvalueD2 == 1)) 
  {
    Serial.println("Both sensors detect line. Moving forward.");
    motorSetForward();
    analogWrite(enB, 74);
    analogWrite(enA, 74);
  } 

  
  //right sensor detects black -- move right
  if (IRvalueD == 1 && IRvalueD2 == 0)
  {
    Serial.println("Right sensor detect line. Moving Right");
    digitalWrite(in1, LOW); //TURN ON LEFT
    digitalWrite(in2, HIGH);
    digitalWrite(in3, LOW); //TURN OFF RIGHT
    digitalWrite(in4, LOW);

    analogWrite(enB, 74);
    analogWrite(enA, 74);
  }

  //left sensor detects black -- move left
  if(IRvalueD == 0 && IRvalueD2 == 1)
  {
    Serial.println("Left sensor detect line. Moving Left");
    digitalWrite(in1, LOW); //TURN OFF LEFT
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW); //TURN ON RIGHT
    digitalWrite(in4, HIGH);

    analogWrite(enB, 74);
    analogWrite(enA, 74);

  }

  //line detected -- don't move
  if (IRvalueD == 0 && IRvalueD2 == 0)
  {
    Serial.println("Line not detected. Stopping.");
    motorBrake();
  }


}



void controlDrive()
{
  val1 = phone.getThrottle() - 49;
  val2 = phone.getSteering() - 49;

  if (val1 == 0 && abs(val2) < 10) // No throttle
  {
    motorBrake();
  }
  else if (val1 > 10) // Move forward
  {
    speed = map(val1, 0, 50, 0, 200);
    Serial.println("Moving Forward");
    motorSetForward();
    analogWrite(enA, speed);
    analogWrite(enB, speed);

    if (val2 > 10) // Turn right while moving forward
    {
      reducedSpeed = map(val2, 0, 50, speed, 0);
      Serial.println("Turning Right");
      analogWrite(enA, speed);
      analogWrite(enB, reducedSpeed);
    }
    else if (val2 < -10) // Turn left while moving forward
    {
      reducedSpeed = map(abs(val2), 0, 50, speed, 0);
      Serial.println("Turning Left");
      analogWrite(enA, reducedSpeed);
      analogWrite(enB, speed);
    }
  }
else if (val1 < -10) // Move backward
{
    speed = map(abs(val1), 0, 50, 0, 200);
    Serial.println("Moving Backward");
    motorSetBackward();
    analogWrite(enA, speed);
    analogWrite(enB, speed);

    if (val2 > 10) // Turn right while moving backward
    {
      reducedSpeed = map(val2, 0, 50, speed, 0);
      Serial.println("Turning Right");
      analogWrite(enA, speed);
      analogWrite(enB, reducedSpeed);
    }
    else if (val2 < -10) // Turn left while moving backward
    {
      reducedSpeed = map(abs(val2), 0, 50, speed, 0);
      Serial.println("Turning Left");
      analogWrite(enA, reducedSpeed);
      analogWrite(enB, speed);
    }
  }

 //Serial.print("Throttle: "); Serial.print(val1);
  //Serial.print("\t"); Serial.print("Steering: "); Serial.println(val2);
}
