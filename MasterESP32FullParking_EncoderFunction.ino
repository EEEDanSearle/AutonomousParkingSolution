
#include <Wire.h>
#include <MPU6050_tockn.h> //MPU library
#define I2C_SLAVE_ADDR 0x04 // 4 in hexadecimal
#define MPU_6050_ADDR 0x68 //MPU address
#define SpeedOfSound 0.0343 //Speed of sound in cm/microsecond

MPU6050 mpu6050(Wire);

const int trigPin = 15; //Define Trigger pin
const int echoPin = 2; //Define Echo pin

void setup()
{
  Serial.begin(9600);
  Wire.begin();   // join i2c bus (address optional for the master) - on the Arduino NANO the default I2C pins are A4 (SDA), A5 (SCL)
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  pinMode(trigPin, OUTPUT);  //Define Trigger as output
  pinMode(echoPin,INPUT);  //Define Echo as input
}

//Change values in loop to manouevre
int x = 0;  //Left Motor
int y = 0;  //Right Motor
int z = 110;  //Servo Angle

long CalculateDistance(uint16_t clicks)  //Define Function Calculate Distance
{
  float diameter = 6,distance;   //Define variable diameter and distance
  distance = (clicks*((M_PI*diameter)/24));   //Calculates distance by piD and dividing by clicks per revolution of the wheel
  return(distance);  // Returns calculated value
}

uint16_t rotaryCount(int EncNum)
{
  Wire.requestFrom(I2C_SLAVE_ADDR,4); //Requests Data from Arduino encoders
  uint16_t Enc1, Enc2;  //Encoder Speed
  uint8_t Enc1_169 = Wire.read();  //Receving encoder bits 16-9
  uint8_t Enc1_81 = Wire.read();  //Receiving encoder bits 8-1
  uint8_t Enc2_169 = Wire.read();
  uint8_t Enc2_81 = Wire.read();
  Enc1 = (Enc1_169 <<8) | Enc1_81;  //Combines two 8 bit numbers into a 16 bit
  Enc2 = (Enc2_169 <<8) | Enc2_81;

  switch(EncNum)
  {
    case 1:
    return(Enc1);
    case 2:
    return(Enc2);
  }
}

float UltrasonicDistance()
{
  float distance,duration; //Define function variables

  digitalWrite(trigPin,LOW); //Reset pin
  delay(1);  //delay of 1ms
  digitalWrite(trigPin,HIGH); //Sets to high for 10microseconds
  delayMicroseconds(10);
  digitalWrite(trigPin,LOW);

  duration = pulseIn(echoPin, HIGH); //Reads Echo pin for High Signal, returns in microseconds

  distance = duration*(SpeedOfSound/2); //Calculates Distance in cm
  return(distance); //Return calculated value to main
}

void ChangeDirection(int leftMotor, int rightMotor, int SteeringAngle)
{
  Wire.beginTransmission(I2C_SLAVE_ADDR); // transmit to device #4
  
  //send leftMotorSpeed
  Wire.write((byte)((leftMotor & 0x0000FF00) >> 8));    // first byte of x, containing bits 16 to 9
  Wire.write((byte)(leftMotor & 0x000000FF));           // second byte of x, containing the 8 LSB - bits 8 to 1
  //Send rightMotorSpeed
  Wire.write((byte)((rightMotor & 0x0000FF00) >> 8));    // first byte of y, containing bits 16 to 9
  Wire.write((byte)(rightMotor & 0x000000FF));           // second byte of y, containing the 8 LSB - bits 8 to 1
  //send steering angle to arduino
  Wire.write((byte)((SteeringAngle & 0x0000FF00) >> 8));    
  Wire.write((byte)(SteeringAngle & 0x000000FF));
 
  Wire.endTransmission();   // stop transmitting
}

int findAngle()
{
  mpu6050.update();//get data from mpu
  int ZAngle = mpu6050.getAngleZ();  //Find Angle Z
  return(ZAngle);
}

void TurnClockwise(int Angle)
{

 int StartAngle = findAngle();
 Serial.println("StartAngle: ");
 Serial.print(StartAngle);
 int AngleTurned = (findAngle() - StartAngle);

 while (AngleTurned <= Angle-10)
 {
   ChangeDirection(200,200,170); //turn Right (clockwise)
   AngleTurned = (findAngle() - StartAngle);
    Serial.println("AngleTurned: ");
    Serial.print(AngleTurned);
    delay(50);
 }
 ChangeDirection(0,0,145); //Stop car
}

void TurnAntiClockwise(int Angle)
{
  int StartAngle = findAngle();
 Serial.println("StartAngle: ");
 Serial.print(StartAngle);
 int AngleTurned = (StartAngle - findAngle());

 while (AngleTurned <= Angle-10)
 {
   ChangeDirection(200,200,110); //turn left (anticlockwise)
   AngleTurned = (StartAngle - findAngle());
   // Serial.println("AngleTurned: ");
   // Serial.print(AngleTurned);
    delay(50);
 }
 ChangeDirection(0,0,145); //Stop car
}

void ReversePark(float distance)
{
  float SensorDistance = UltrasonicDistance();
  int StartAngle = findAngle();
  while (distance+16.5 <= SensorDistance)
  {
    int actualAngle = findAngle();
    Serial.print("actualangle: ");
    Serial.println(actualAngle);
    if (actualAngle != StartAngle)

    {
      int Turn = StartAngle - actualAngle;
      ChangeDirection(-255,-255,145-Turn);
    }
    else ChangeDirection(-255,-255,145);
    SensorDistance = UltrasonicDistance();
    Serial.print("Distance: ");
    Serial.println(SensorDistance);
    delay(50);
  }
  ChangeDirection(0,0,145); //stop car
}

void TravelForwards(float Time, int StartAngle)
{
  time_t start,end;
  float elapsed;
  time(&start);

  while (elapsed <= Time)
  {
    int actualAngle = findAngle();
    Serial.print("actualangle: ");
    Serial.println(actualAngle);
    if (actualAngle != StartAngle)

    {
      int Turn = StartAngle - actualAngle;
      ChangeDirection(255,255,145+Turn);
    }
    else ChangeDirection(255,255,145);
    
    time(&end);
    elapsed = difftime(end,start);
    delay(50);
   
  }
  ChangeDirection(0,0,145); //stop car
}

void TravelBackwards(float Time, int StartAngle)
{
  time_t start,end;
  float elapsed;
  time(&start);

  while (elapsed <= Time)
  {
    int actualAngle = findAngle();
    Serial.print("actualangle: ");
    Serial.println(actualAngle);
    if (actualAngle != StartAngle)

    {
      int Turn = StartAngle - actualAngle;
      ChangeDirection(-255,-255,145-Turn);
    }
    else ChangeDirection(-255,-255,145);
    
    time(&end);
    elapsed = difftime(end,start);
    delay(50);
   
  }
  ChangeDirection(0,0,145); //stop car
}

void loop()
{
  uint16_t D1 = rotaryCount(1);
  uint16_t D2 = rotaryCount(2);

  float WallDistance = UltrasonicDistance(); //Gets data from ultrasonic sensor
  int AngleTurned = findAngle(); //Gets angle from mpu

  
  TravelForwards(1,findAngle());  //Travel Forwards for 1 second, providing the initial angle
  //delay(1000);
  ChangeDirection(0,0,145); //Stops Car

  //Turn Through an angle of 180 degrees in a clockwise direction
  TurnClockwise(180);

  ReversePark(10);  //reverse up to a surface 10 cm away

  TurnClockwise(90); //Turn 90 degrees clockwise

  ReversePark(10); //Reverse up to another surface, leaving a 10cm gap




  
  
  Serial.print("Angle = ");  //Output ZAngle
  Serial.println(findAngle());

  Serial.print("Enc 1 = ");  //Output data from encoders
  Serial.println(D1);
  Serial.print("Enc 2 = ");
  Serial.println(D2);

  Serial.print("Ultrasonic Distance = ");
  Serial.println(WallDistance);

  /*Serial.print("D1 = ");
  Serial.println(CalculateDistance(Enc1));
  Serial.print("D2 = ");
  Serial.println(CalculateDistance(Enc2));*/

  


  

  delay(10000);
}
