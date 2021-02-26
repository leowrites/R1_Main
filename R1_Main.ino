#include <MPU9250.h>
#include <Adafruit_BMP280.h>
#include <SD.h>
#include <SPI.h>
#include <Servo.h>

/*

Software for R1

Leo Liu
Feb 19, 2021

*/

#ifndef YAW_SERVO_PIN
#define YAW_SERVO_PIN 2
#endif
#ifndef PITCH_SERVO_PIN
#define PITCH_SERVO_PIN 3
#endif
#ifndef BUTTON_PIN
#define BUTTON_PIN 4
#endif
#ifndef SERVO_DEFAULT
#define SERVO_DEFAULT 90
#endif
#ifndef SERVO_MAX
#define SERVO_MAX 100
#endif
#ifndef SERVO_MIN
#define SERVO_MIN 80
#endif
#ifndef GLOBAL_DELAY
#define GLOBAL_DELAY 100
#endif
#ifndef INITIAL_MASS
#define INITIAL_MASS 2000
#endif
#ifndef MASS_LOSS_RATE
//per booster for 7 seconds
#define MASS_LOSS_RATE 5.17
#endif
#ifndef INTERVAL_DELAY
#define INTERVAL_DELAY 20
#endif
#ifndef DEFAULT_ORIENTATION
#define DEFAULT_ORIENTATION 0
#endif
#ifndef CS
#define CS 10 //change when I get the shield
#endif
#ifndef CALC_INTERVAL
#define CALC_INTERVAL 20
#endif
#ifndef DATA_INTERVAL
#define DATA_INTERVAL 100
#endif

byte xAdj = 0;
byte yAdj = 0;
short int currentMass;
unsigned long LAUNCH_TIME;
unsigned long timeSinceLaunch;
unsigned long *prev_ms_calc = 0;
unsigned long *prev_ms_data = 0;
//use byte to indicate status to save memory
byte flightStatus = 0;
bool launched = false;

float pGx = 0.0f;
float pGy = 0.0f;
float dx = 0.0f;
float dy = 0.0f;
//just declare everything global lol Im done with this
float orientation[1];
float gyro_rate[1];
float dG[1];
float bmp_data[2];
//delta T

struct servo
{
  Servo servoNum;
  unsigned int pin;
  int servoRotation;
  bool hor;
  bool ver;
  bool hor_max_confirmed;
  bool hor_min_confirmed;
  bool ver_max_confirmed;
  bool ver_min_confirmed;
};

Servo yawServo;
Servo pitchServo;
Adafruit_BMP280 bmpSensor;
MPU9250 imuSensor;
File dataFile;

struct servo servo1A
{
  .servoNum = yawServo, .pin = YAW_SERVO_PIN, .servoRotation = SERVO_DEFAULT, .hor = true, .ver = false
};
struct servo servo1B
{
  .servoNum = pitchServo, .pin = PITCH_SERVO_PIN, .servoRotation = SERVO_DEFAULT, .hor = false, .ver = true
};
struct servo servoArray[] = {servo1A, servo1B};

//function prototyping
bool check_time(unsigned long *last_ms, unsigned short int interval);
int preFlightHold();
void preFlightCheck();
void launchCountDown();
void ignition();
void inFlight();
void gyroReading();
void bmpDataReading();
void writeData();
void adjustmentCalculation();
void orientationIntegration();
void proportionalGain();
void derivativeConvertion();
void angleOutput();
void calculate_mass();
void resetServo();
void thrustVectorControl(servo servo, bool hor);
void parachuteDepolyment(float pressure, float attitude);
void servo_print(short int result, byte servoPin);
void raise_initialization_error(String msg);

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println(F("System initializing"));
  Wire.begin();

  pinMode(BUTTON_PIN, INPUT);
  yawServo.attach(YAW_SERVO_PIN);
  pitchServo.attach(PITCH_SERVO_PIN);

  currentMass = INITIAL_MASS;

  if (!bmpSensor.begin())
  {
    raise_initialization_error(F("BMP sensor"));
  }
  else
  {
    Serial.println(F("BMP initialization successful."));
  }

  if (!imuSensor.setup(0x68))
  {
    raise_initialization_error(F("IMU sensor"));
  }
  else
  {
    Serial.println(F("IMU initialization successful."));
  }
  if (!SD.begin(CS))
  {
    raise_initialization_error(F("SD card"));
  }
  else
  {
    Serial.println(F("SD initializatoin successful."));
  }

  dataFile = SD.open(F("datalog.txt"), FILE_WRITE);

  if (!dataFile)
  {
    raise_initialization_error(F("SD card file "));
  }

  dataFile.println(F("------------ this is the beginning of the data recording ------------"));
}

bool check_time(unsigned long *last_ms, unsigned short int interval)
{
  if (millis() - *last_ms >= interval)
  {
    *last_ms = millis();
    return true;
  }
  else
  {
    return false;
  }
}

int preFlightHold()
{
  //await signal for pre-flight confirmation
  //bluetooth or a button?
  int buttonState = digitalRead(BUTTON_PIN);
  bool message_sent = false;
  while (buttonState == LOW)
  {
    buttonState = digitalRead(BUTTON_PIN);
  }
  Serial.println(F("Flight confirmed"));
}

void preFlightCheck()
{
  //check the rotation of the servos, readings from bmp and imu, and sd card recording, recalibrate bmp and imu
  resetServo();
  Serial.println(F("IMU calibration begin."));
  imuSensor.verbose(true);
  imuSensor.calibrateAccelGyro();
  imuSensor.calibrateMag();
}

void launchCountDown()
{
  //switch case to avoid blocking code
  unsigned char tSeconds = 60;
  while (tSeconds > 0)
  {
    tSeconds--;
    delay(1000);
    if (tSeconds == 5)
    {
      //initiate calculation thread and tvc at t-5
      adjustmentCalculation();
    }
  }
  //activate the inFlight loop in a seperate thread at T-1
}

void ignition()
{
  //outputs power to ignite the rocket motors
  LAUNCH_TIME = millis();
  launched = true;
}

void inFlight()
{
  //this will be a data recording loop and transition to parachute depolyment.
  //calculations will be done in another thread
  while (true)
  { //replace with pressure sensor
  }
}

//data reading

void gyroReading()
{
  //return in rad/20ms
  float dx = imuSensor.getGyroX();
  float dy = imuSensor.getGyroY();
  dx = 20 * dx / 1000;
  dy = 20 * dy / 1000;

  gyro_rate[0] = dx;
  gyro_rate[1] = dy;
}

void bmpDataReading()
{
  bmp_data[0] = bmpSensor.readPressure();
  bmp_data[1] = bmpSensor.readAltitude();
  bmp_data[2] = bmpSensor.readTemperature();
}

//data recording
void writeData()
{
  dataFile.println(millis());
  dataFile.print(F("X rate of change: "));
  dataFile.println(gyro_rate[0]);
  dataFile.print(F("Y rate of change: "));
  dataFile.println(gyro_rate[1]);
}

//data processing

void adjustmentCalculation()
{
  //updates the adjustment angles
  //current orientation (same as error)
  //loop executes once every 20 miliseconds
  //the array returning probably need fixing
  gyroReading();
  orientationIntegration();
  proportionalGain();
  derivativeConvertion();
  angleOutput();
  //call tvc
  thrustVectorControl(servo1A, servo1A.hor);
  thrustVectorControl(servo1B, servo1B.hor);
}

void orientationIntegration()
{
  //integrate radian/s with respect to mili seconds
  //differentiate gain and lost
  orientation[0] += gyro_rate[0];
  orientation[1] += gyro_rate[1];
}

void proportionalGain()
{
  pGx = orientation[0] * 0.6;
  pGy = orientation[1] * 0.6;
}

void derivativeConvertion()
{
  dx = gyro_rate[0] * 0.3;
  dy = gyro_rate[1] * 0.3;
}

void angleOutput()
{
  xAdj = pGx + dx;
  yAdj = pGy + dy;
}

void calculate_mass()
{
  //calculates mass
  timeSinceLaunch = millis() - LAUNCH_TIME;
  currentMass = INITIAL_MASS - MASS_LOSS_RATE * timeSinceLaunch;
}

//servo operation

void resetServo()
//resets the rotation of all servos
{
  for (size_t i = 0; i < sizeof(servoArray) / sizeof(servoArray[0]); i++)
  {
    servo servo = servoArray[i];
    servo.servoRotation = servo.servoNum.read();
    if (servo.servoRotation > 90)
    {
      while (servo.servoRotation > 90)
      {
        servo.servoRotation--;
        servo.servoNum.write(servo.servoRotation);
      }
    }
    else if (servo.servoRotation < 90)
    {
      while (servo.servoRotation < 90)
      {
        servo.servoRotation++;
        servo.servoNum.write(servo.servoRotation);
      }
    }
    delay(GLOBAL_DELAY);
  }
  Serial.println("Servos at default.");
}

void thrustVectorControl(servo servo, bool hor)
{
  //controls indiviudal servos
  //move to exact position
  servo.servoRotation = servo.servoNum.read();
  if (hor)
  {
    short int current = servo.servoNum.read();
    short int result = current + xAdj;
    servo.servoNum.write(result);
    servo_print(result, servo.pin);
  }
  else
  {
    short int current = servo.servoNum.read();
    short int result = current + yAdj;
    servo.servoNum.write(result);
    servo_print(result, servo.pin);
  }
}

//parachute

void parachuteDepolyment(float pressure, float attitude)
{
  //cross reference
}

//feedback to user

void servo_print(short int result, byte servoPin)
{
  Serial.print(F("Servo:"));
  Serial.print(servoPin);
  Serial.print(F(" rotation: "));
  Serial.println(result);
}

void raise_initialization_error(String msg)
{
  Serial.println(msg + F(" initialization failed"));
}

void loop()
{
  //state manager
  //switch
  if (!launched)
  {
    preFlightHold();
    preFlightCheck();
    launchCountDown();
    ignition();
    flightStatus = 1; // 1 = inflight, 0 = not in flight
    while (flightStatus = 1)
    {
      if (check_time(prev_ms_calc, CALC_INTERVAL))
      {
        adjustmentCalculation();
      }
      if (check_time(prev_ms_data, DATA_INTERVAL))
      {
        //record data function
      }
    }
    float pressure = bmpSensor.readPressure();
    float attitude = bmpSensor.readAltitude();
    parachuteDepolyment(pressure, attitude);
    dataFile.close();
  }
}
