/*

Software for R1

Leo Liu
Feb 19, 2021

*/
#include <MPU9255.h>
#include <Servo.h>
#include <Adafruit_BMP280.h>
#include <SD.h>
#include <SPI.h>
#include <Servo.h>

#define g 9.81
#define YAW_SERVO_PIN 2
#define PITCH_SERVO_PIN 3
#define SDCS 4
#define BUTTON_PIN 5
#define servoX_offset 90
#define servoY_offset 90
#define SERVO_MAX 130
#define SERVO_MIN 50
#define GLOBAL_DELAY 100
#define INITIAL_MASS 2000
#define MASS_LOSS_RATE 5.17
#define BURN_OUT_TIME 7000

short int currentMass;
unsigned long launchTime;
unsigned long timeSinceLaunch;

byte flightStatus = 0;
bool launched = false;
bool parachute_depolyed = false;
bool motor_burn_out = false;

//PID variables
double PIDX, PIDY, errorX, errorY, previous_errorX, previous_errorY, pwmX, pwmY, gyroXX, gyroYY;
//post processing
double pax, pay, paz, pgx, pgy, pgz;
//angle output
int pmw, pmy;
//error
int desired_angleX, desired_angleY = 0;
//pid
double accAngleX, accAngleY, yaw, pitch, gyroAngleX, gyroAngleY, dt, cur_time, prev_time;
float servo_gear_ratio = 5.8;
float pidX_p = 0;
float pidX_i = 0;
float pidX_d = 0;
float pidY_p = 0;
float pidY_i = 0;
float pidY_d = 0;
double kp = 0.13;
double ki = 0.0;
double kd = 0.05;

float pressure;
float attitude;

Adafruit_BMP280 bmpSensor;
MPU9255 mpu;
File dataFile;
Servo servoX;
Servo servoY;

int preFlightHold()
{
  //await signal for pre-flight confirmation
  //bluetooth or a button?
  int buttonState = digitalRead(BUTTON_PIN);
  while (buttonState == LOW)
  {
    buttonState = digitalRead(BUTTON_PIN);
    Serial.print(F("Awaiting confirmatiom."));
  };
  dataFile.print("Time: ");
  dataFile.print(millis());
  dataFile.println(F("Flight confirmed."));
}

void launchCountDown()
{
  //switch case to avoid blocking code
  unsigned char tSeconds = 60;
  while (tSeconds > 0)
  {
    tSeconds--;
    delay(1000);
  }
}

void ignition()
{
  //outputs power to ignite the rocket motors
  launchTime = millis();
  launched = true;
  flightStatus = 1;
}

void inFlight()
{
  //this will be a data recording loop and transition to parachute depolyment.
  //calculations will be done in another thread
  Serial.println(F("TVC Loop"));
  timeSinceLaunch = millis() - launchTime;
  if (timeSinceLaunch > BURN_OUT_TIME)
  {
    motor_burn_out = true;
  }
  if (!motor_burn_out)
  {
    calculate_mass();
  }
  read_data();
  process_data();
  delta_time();
  calculate_accel_degree();
  pid();
  servo_write();
  pressure = bmpSensor.readPressure();
  attitude = bmpSensor.readAltitude();
  write_to_sd();
  parachuteDepolyment();
}

void landing()
{
  //check y acceleration if it is the same as gravity then stop data recording
  if (pay < 10 && motor_burn_out == true && parachute_depolyed && true)
  {
    dataFile.print("Landing complete. ");
    dataFile.close();
  }
  else
  {
    write_to_sd();
  }
}

//servo operation
void reset_servo()
{
  servoX.write(servoX_offset);
  servoY.write(servoY_offset);
}

double process_acceleration(int input)
{
  // process raw acceleration
  double output = 1;

  //for +- 2g
  output = input;
  output = output / 16384;
  output = output * g;

  return output;
}

double process_angular_velocity(int16_t input)
{
  //process raw gyro rate

  //for +- 250 dps
  return input / 131;
}

void delta_time()
{
  prev_time = cur_time;
  cur_time = millis();
  dt = (cur_time - prev_time) / 1000;
}

void read_data()
{
  Serial.println(F("Reading Data"));
  mpu.read_acc();
  mpu.read_gyro();
  mpu.read_mag();
}

void process_data()
{
  pax = process_acceleration(mpu.ax);
  pay = process_acceleration(mpu.ay);
  paz = process_acceleration(mpu.az);
  pgx = process_angular_velocity(mpu.gx);
  pgy = process_angular_velocity(mpu.gy);
  pgz = process_angular_velocity(mpu.gz);

  if (-5 <= pgx && pgx <= 0 || 0 <= pgx && pgx <= 5)
  {
    pgx = 0;
  };
  if (-5 <= pgy && pgy <= 0 || 0 <= pgy && pgy <= 5)
  {
    pgy = 0;
  };
  if (-5 <= pgz && pgz <= 0 || 0 <= pgz && pgz <= 5)
  {
    pgz = 0;
  };

  Serial.print(F("AX: "));
  Serial.print(pax);
  Serial.print(F("  AY: "));
  Serial.print(pay);
  Serial.print(F("  AZ: "));
  Serial.print(paz);
  Serial.print(F("      GX: "));
  Serial.print(pgx);
  Serial.print(F("  GY: "));
  Serial.print(pgy);
  Serial.print(F("  GZ: "));
  Serial.println(pgz);
}

void write_to_sd()
{
  if (dataFile)
  {
    dataFile.print("Time: ");
    dataFile.println((millis()));
    dataFile.print(F("AX: "));
    dataFile.print(pax);
    dataFile.print(F("  AY: "));
    dataFile.print(pay);
    dataFile.print(F("  AZ: "));
    dataFile.print(paz);
    dataFile.print(F("      GX: "));
    dataFile.print(pgx);
    dataFile.print(F("  GY: "));
    dataFile.print(pgy);
    dataFile.print(F("  GZ: "));
    dataFile.println(pgz);
    dataFile.print(F("Pressure: "));
    dataFile.print(pressure);
    dataFile.print(F("  Attitude: "));
    dataFile.print(F("Pitch: "));
    dataFile.print(pitch);
    dataFile.print(F("    Yaw:"));
    dataFile.println(yaw);
    dataFile.println(attitude);
    dataFile.print(F("PIDX: "));
    dataFile.println(PIDX);
  }
}

void calculate_accel_degree()
{

  double prevgyroX = pgx;
  double prevgyroY = pgz;

  gyroAngleX += (((pgx + prevgyroX) / 2) * dt); // deg/s * s = deg
  gyroAngleY += (((pgz + prevgyroY) / 2) * dt);

  double orientationX = 0.90 * gyroAngleX + 0.10 * accAngleX;
  double orientationY = 0.90 * gyroAngleY + 0.10 * accAngleY;

  //divided by 32.8 as recommended by the datasheet
  pitch = orientationX + 2;
  yaw = orientationY + 2;

  Serial.print(F("Pitch: "));
  Serial.print(pitch);
  Serial.print(F("    Yaw:"));
  Serial.println(yaw);
}

void pid()
{

  previous_errorX = errorX;
  previous_errorY = errorY;

  errorX = pitch - desired_angleX;
  errorY = yaw - desired_angleY;

  pidX_p = kp * errorX;
  pidY_p = kp * errorY;

  pidX_d = kd * ((errorX - previous_errorX) / dt);
  pidY_d = kd * ((errorY - previous_errorY) / dt);

  pidX_i = ki * (pidX_i + errorX * dt);
  pidY_i = ki * (pidY_i + errorY * dt);

  PIDX = pidX_p + pidX_i + pidX_d;
  PIDY = pidY_p + pidY_i + pidX_d;

  Serial.print(F("PIDX: "));
  Serial.println(PIDX);

  //constraint for pwm
  pwmX = ((PIDX * servo_gear_ratio) + servoX_offset);
  pwmY = ((PIDY * servo_gear_ratio) + servoY_offset);
}

void servo_write()
{

  servoX.write(pwmX);
  servoY.write(pwmY);
}

void calculate_mass()
{
  currentMass = INITIAL_MASS - (timeSinceLaunch / 1000 * MASS_LOSS_RATE);
  dataFile.print("Mass: ");
  dataFile.println(currentMass);
}

void parachuteDepolyment()
{
  //cross reference y acceleration and motor burn out time
  if (motor_burn_out)
  {
    parachute_depolyed = true;
    dataFile.print("Motor burn out, Time: ");
    dataFile.println(millis());
  }
}

void raise_initialization_error(String msg)
{
  Serial.println(msg + F(" initialization failed"));
  while (1)
    ;
}

void setup()
{
  // put your setup code here, to run once:
  pinMode(BUTTON_PIN, INPUT);

  Serial.begin(9600);
  Serial.println(F("System initializing"));
  servoX.attach(YAW_SERVO_PIN);
  servoY.attach(PITCH_SERVO_PIN);

  currentMass = INITIAL_MASS;

  if (!bmpSensor.begin())
  {
    raise_initialization_error(F("BMP sensor"));
  }
  else
  {
    Serial.println(F("BMP initialization successful."));
  }

  if (mpu.init())
  {
    raise_initialization_error(F("IMU sensor"));
  }
  else
  {
    Serial.println(F("IMU initialization successful."));
  }
  if (!SD.begin(SDCS))
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
    raise_initialization_error(F("SD card file writing"));
  }

  dataFile.println(F("------------ this is the beginning of the data recording ------------"));
  dataFile.println(millis());
}

void loop()
{
  while (!launched)
  {
    preFlightHold();
    launchCountDown();
    ignition();
  }
  while (flightStatus = 1)
  {
    inFlight();
    if (parachute_depolyed)
    {
      flightStatus = 2;
    }
  }
  while (flightStatus = 2)
  {
    landing();
  }
}
