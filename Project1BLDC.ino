// Magnetic Encoder Stuff
#include <Wire.h>
int magnetStatus=0;
int lowbyte;
word highbyte;
int rawAngle; //Final Raw Angle
float degAngle; // Final Raw angle in degrees (360/4096 * (measured between 0-4096))
float correctedAngle = 0; // zerod angle - startup value
float startAngle = 0;

// PID Stuff
#include <PID_v1_bc.h>
double Kp = 110, Ki = 191, Kd = 0; // 100, 94, 0 | -124, -192, 0
double PendulumAngle, MotorSpeed, DesiredAngle = 0;
PID ReactionPID(&PendulumAngle, &MotorSpeed, &DesiredAngle, Kp, Ki, Kd, REVERSE);

// BLDC Stuff
#include <Servo.h>
Servo ESC;
float bldcSpeed;
int count;

void setup() {
  Serial.begin(115200);
// Encoder I2C
  Wire.begin();
        Wire.setClock(800000L);
  checkMagnetPresence();
  ReadRawAngle();
  startAngle = degAngle;
// PID Stuff
  ReactionPID.SetMode(AUTOMATIC);
  ReactionPID.SetOutputLimits(-255,255);
// BLDC Stuff
  ESC.attach(9,1000,3000);
}

void loop() {
  ReadRawAngle();
  correctAngle();
  PendulumAngle = -correctedAngle;
  ReactionPID.Compute();
  Serial.print("PID Speed: ");
  Serial.print(MotorSpeed);
  Serial.print(" BLDC Speed: ");
  Serial.print(bldcSpeed);
  Serial.print(" Pendulum Angle: ");
  Serial.println(PendulumAngle);
  // Safety mechanism to limit max/min speed, just in case.
  if(MotorSpeed<-255)
  {
    MotorSpeed = -255;
  }
  if(MotorSpeed>255)
  {
    MotorSpeed = 255;
  }

  bldcSpeed = map(MotorSpeed, -255,  255, 0, 177);
  ESC.write(bldcSpeed);
/*
  // Set direction of motor spin
  if(MotorSpeed<0)
  {
    motor.run(BACKWARD);
    motor.setSpeed(abs(MotorSpeed));
  }
  else
  {
    motor.run(FORWARD);
    motor.setSpeed(abs(MotorSpeed));
  }
*/
}

void ReadRawAngle()
{
  // 7:0 bits out of 12 bit measurement
  Wire.beginTransmission(0x36); // ID of sensor
  Wire.write(0x0D);
  Wire.endTransmission();
  Wire.requestFrom(0x36, 1);

  while(Wire.available() == 0);
  lowbyte = Wire.read();

  // 11:8 bits out of 12 bit measurement
  Wire.beginTransmission(0x36);
  Wire.write(0x0C);
  Wire.endTransmission();
  Wire.requestFrom(0x36, 1);

    while(Wire.available() == 0);
  highbyte = Wire.read();
// shifting the high byte to the left by 8 as we are trying to combine low and high to make a 12bit number
  highbyte = highbyte << 8;

  rawAngle = highbyte|lowbyte;
// 360/4096 = 0.087890625
  degAngle = rawAngle * 0.087890625;
}
// PROBABLY DON'T NEED THIS NEXT FUNCTION
void correctAngle()
{
  correctedAngle = degAngle-startAngle; // zeroing the angle
/*
  if (correctedAngle < 0)
  {
    correctedAngle = correctedAngle+360;
  }
  else
  {}
*/
}

void checkMagnetPresence()
{
  while((magnetStatus & 32) != 32)
  {
    magnetStatus = 0;

    Wire.beginTransmission(0x36);
    Wire.write(0x0B);
    Wire.endTransmission();
    Wire.requestFrom(0x36, 1);

    while(Wire.available() == 0);
    magnetStatus = Wire.read();

    Serial.print("Magnet status: ");
    Serial.println(magnetStatus, BIN);
  }
}

