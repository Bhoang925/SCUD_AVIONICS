// Initialization
#include "Adafruit_BMP3XX.h"
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <PID_v1.h>
#include <Servo.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

//#define OUTPUT_READABLE_QUATERNION
//#define OUTPUT_READABLE_EULER
#define OUTPUT_READABLE_YAWPITCHROLL
//#define OUTPUT_READABLE_REALACCEL
//#define OUTPUT_READABLE_WORLDACCEL

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13       // (Arduino is 13, Teensy is 11, Teensy++ is 6)
#define x_servo_pin 8
#define y_servo_pin 7
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;   // set true if DMP init was successful
uint8_t mpuIntStatus;    // holds actual interrupt status byte from MPU
uint8_t devStatus;       // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;     // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;      // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];  // FIFO storage buffer

// orientation/motion vars
Quaternion q;         // [w, x, y, z]         quaternion container
VectorInt16 aa;       // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;   // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;  // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;  // [x, y, z]            gravity vector
float euler[3];       // [psi, theta, phi]    Euler angle container
float ypr[3];         // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// ================================================================
// ===                      PID ROUTINE                         ===
// ================================================================

// PID control vars
Servo x_servo, y_servo;

double dt;
double integral, previous, output = 0;  //output initialized to zero
double kp = 1;
double ki = 0.0001;
double kd = 0.001;  // PID gains
double setpoint = 0;
double previousTime = 0;
// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;  // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}
// SD initiliazation
#include <SPI.h>
#include <SD.h>
File datafile;
// LED initialization
#define LED_pin 13
// BMP390 initialization
#define BMP_CS 10
#define SEALEVELPRESSURE_HPA (1013.25)
// Buzzer
#define buzz_pin 37
// Variable assignment
#define ASCENT_ACCEL_THRESHOLD 13
#define APOGEE_ACCEL_THRESHOLD 10
#define CHUTE_DEPLOY_ALTITUDE 25

// MPU preset variables
long accelX, accelY, accelZ;
long gyroX, gyroY, gyroZ;
float rotX, rotY, rotZ;
float real_accel_x, real_accel_y, real_accel_z;

Adafruit_BMP3XX bmp;


int buzzer = buzz_pin;
int yellow_led = LED_pin;
int ascent_accel_threshold = ASCENT_ACCEL_THRESHOLD;
int apogee_accel_threshold = APOGEE_ACCEL_THRESHOLD;
int chute_deployment_altitude = CHUTE_DEPLOY_ALTITUDE;
unsigned long lastMillis = 0;
const int chipSelect = BUILTIN_SDCARD;
pi_numeric = 3.14159265;

// Setup
void setup() {
  pinMode(yellow_led, OUTPUT);
  pinMode(buzzer, OUTPUT);
  Serial.begin(115200);
  Wire.begin();
  mpu_init();
  bpu_startup();
  SD_startup();
  init_servo();
  blink_twice();
  delay(1000);
}
void loop() {
  //avionics_states();
  //tvc_update();
  float y, p, r;
  getYawPitchRoll(y ,p , r);
  Serial.print(y);
  Serial.print("\t");
  Serial.print(p);
  Serial.print("\t");
  Serial.println(r);
  
}

// Avionic States Function
void avionics_states() {

  enum class avionic_State : uint8_t {

    Ground_Idle,        //defaults to 0
    Powered_Flight,     //defaults to 1
    Unpowered_Flight,   //defaults to 2
    Ballistic_Descent,  //defaults to 3
    Chute_Descent,      //defaults to 4
    Landing_Safe,       //defaults to 5

  };
  //Keeping track of current state
  static avionic_State current_State = avionic_State::Ground_Idle;

  switch (current_State) {
      // Ground Idle State
    case avionic_State::Ground_Idle:
      tone(buzzer, 261);
      digitalWrite(yellow_led, HIGH);
      displayState("Ground Idle");
      slow_data_log();
      tvc_update();


      {
        if (real_accel_z >= ascent_accel_threshold) {
          static unsigned long previousTime = millis();
          if (real_accel_z >= ascent_accel_threshold && (millis() - previousTime) > 100) {
            Serial.print("Z Acceleration (m/s^2): ");
            Serial.println(real_accel_z);
            noTone(buzzer);
            current_State = avionic_State::Powered_Flight;
          } else {
            current_State = avionic_State::Ground_Idle;
          }
        }
      }
      break;
      // Powered Flight State
    case avionic_State::Powered_Flight:


      displayState("Powered Flight");
      fast_data_log();
      tvc_update();
      {
        // TVC CODE GOES HERE!


        if (real_accel_z <= apogee_accel_threshold) {
          static unsigned long previousTime = millis();
          if (real_accel_z <= apogee_accel_threshold && (millis() - previousTime) > 100) {
            Serial.print("Main Engine Cut-Off Acceleration (m/s^2): ");
            Serial.println(real_accel_z);
            current_State = avionic_State::Unpowered_Flight;
          }
        }
      }
      break;
      // Unpowered Flight State
    case avionic_State::Unpowered_Flight:

      displayState("Unpowered Flight");
      x_servo.write(90);
      y_servo.write(90);
      {

        // Need altimeter readings
        static int altitude_1_second_in_past = bmp.readAltitude(SEALEVELPRESSURE_HPA);
        delay(1000);
        if (bmp.readAltitude(SEALEVELPRESSURE_HPA) < altitude_1_second_in_past) {
          Serial.println("Rocket is Descending");
          current_State = avionic_State::Ballistic_Descent;
        } else {
          current_State = avionic_State::Unpowered_Flight;
        }
      }
      break;

      // Ballistic Descent State
    case avionic_State::Ballistic_Descent:

      blink_twice();
      displayState("Ballistic Descent");
      fast_data_log();

      // Fire pyro charges for chutes and recovery
      if (bmp.readAltitude(SEALEVELPRESSURE_HPA) < chute_deployment_altitude) {
        //digitalWrite(pin, HIGH); (deploy chutes)
        Serial.println("Chutes Deployed!");
        current_State = avionic_State::Chute_Descent;
      } else {
        current_State = avionic_State::Ballistic_Descent;
      }
      break;
      // Chute Descent State
    case avionic_State::Chute_Descent:

      displayState("Chute Descent");
      fast_data_log();

      if (current_State == avionic_State::Chute_Descent) {
        static unsigned long previousTime = millis();
        int chute_descent_time_1 = bmp.readAltitude(SEALEVELPRESSURE_HPA);
        if (millis() - previousTime > 5000) {
          int chute_descent_time_2 = bmp.readAltitude(SEALEVELPRESSURE_HPA);
          if (chute_descent_time_1 == chute_descent_time_2) {
            current_State = avionic_State::Landing_Safe;
          } else {
            current_State = avionic_State::Chute_Descent;
          }
        }
      }
      break;

      // Landing Safe State
    case avionic_State::Landing_Safe:

      digitalWrite(yellow_led, HIGH);
      displayState("Landing Safe");

      while (1) {
      }
      // Commence Shutdown of all electronics
  }
}

// Display function for the States
void displayState(String currState) {
  static String prevState = "";

  if (currState != prevState) {
    Serial.println(currState);
    prevState = currState;
  }
}

void bpu_startup() {
  while (!Serial)
    ;
  if (!bmp.begin_I2C()) {  // hardware I2C mode, can pass in address & alt Wire
    //if (! bmp.begin_SPI(BMP_CS)) {  // hardware SPI mode
    //if (! bmp.begin_SPI(BMP_CS, BMP_SCK, BMP_MISO, BMP_MOSI)) {  // software SPI mode
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1)
      ;
  }

  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
}

void SD_startup() {
  while (!Serial) {
    ;  // wait for serial port to connect.
  }

  Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    while (1) {
      // No SD card, so don't do anything more - stay stuck here
    }
  }
  Serial.println("card initialized.");
}

void slow_data_log() {
  String dataString = "";
  unsigned long time = millis();

  if ((time - lastMillis) >= 200) {
    dataString += String(real_accel_z);
    dataString += ",";
    dataString += String(bmp.readAltitude(SEALEVELPRESSURE_HPA));

    File dataFile = SD.open("tvc.txt", FILE_WRITE);

    if (dataFile) {
      dataFile.println(dataString);
      dataFile.close();
      lastMillis = time;
    }
    // if the file isn't open, pop up an error:
    else {
      Serial.println("error opening tvc.txt");
    }
  }
}

void fast_data_log() {
  String dataString = "";
  unsigned long time = millis();

  if ((time - lastMillis) >= 50) {
    dataString += String(real_accel_z);
    dataString += ",";
    dataString += String(bmp.readAltitude(SEALEVELPRESSURE_HPA));

    File dataFile = SD.open("tvc.txt", FILE_WRITE);

    if (dataFile) {
      dataFile.println(dataString);
      dataFile.close();
      lastMillis = time;
    }
    // if the file isn't open, pop up an error:
    else {
      Serial.println("error opening tvc.txt");
    }
  }
}

void recordAccelRegisters() {
  Wire.beginTransmission(0b1101000);  //I2C address of the MPU
  Wire.write(0x3B);                   //Starting register for Accel Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000, 6);  //Request Accel Registers (3B - 40)
  while (Wire.available() < 6)
    ;
  accelX = Wire.read() << 8 | Wire.read();  //Store first two bytes into accelX
  accelY = Wire.read() << 8 | Wire.read();  //Store middle two bytes into accelY
  accelZ = Wire.read() << 8 | Wire.read();  //Store last two bytes into accelZ
  process_accel_data();
}

void recordGyroRegisters() {
  Wire.beginTransmission(0b1101000);  //I2C address of the MPU
  Wire.write(0x43);                   //Starting register for Gyro Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000, 6);  //Request Gyro Registers (43 - 48)
  while (Wire.available() < 6)
    ;
  gyroX = Wire.read() << 8 | Wire.read();  //Store first two bytes into accelX
  gyroY = Wire.read() << 8 | Wire.read();  //Store middle two bytes into accelY
  gyroZ = Wire.read() << 8 | Wire.read();  //Store last two bytes into accelZ
  processGyroData();
}

void process_accel_data() {
  real_accel_x = accelX / 16384.0 * 9.81;
  real_accel_y = accelY / 16384.0 * 9.81;
  real_accel_z = accelZ / 16384.0 * 9.81;
}

void processGyroData() {
  rotX = gyroX / 131.0;
  rotY = gyroY / 131.0;
  rotZ = gyroZ / 131.0;
}

void blink_twice() {
  for (int x = 0; x < 2; x++) {
    digitalWrite(yellow_led, HIGH);
    delay(100);
    digitalWrite(yellow_led, LOW);
    delay(100);
  }
}

void init_servo() {
  x_servo.attach(x_servo_pin);
  y_servo.attach(y_servo_pin);
  x_servo.write(90);
  y_servo.write(90);
}

void mpu_init() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    //Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif

  // initialize serial communication
  // (115200 chosen because it is required for Teapot Demo output, but it's
  // really up to you depending on your project)
  Serial.begin(115200);
  while (!Serial)
    ;  // wait for Leonardo enumeration, others continue immediately

  // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
  // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
  // the baud timing being too misaligned with processor ticks. You must use
  // 38400 or slower in these cases, or use some kind of external separate
  // crystal solution for the UART timer.

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXAccelOffset(-131);  // 1688 factory default for my test chip
  mpu.setYAccelOffset(90);
  mpu.setYAccelOffset(67);
  mpu.setXGyroOffset(-2162);
  mpu.setYGyroOffset(-2923);
  mpu.setZGyroOffset(1391);
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

}

void getYawPitchRoll(float y, float p, float r) {
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {  // Get the Latest packet
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      
      float q0 = q.w;
      float q1 = q.x;
      float q2 = q.y;
      float q3 = q.z;

      float yr = -atan2(-2 * q1 * q2 + 2 * q0 * q3, q2 * q2 - q3 * q3 *q3 - q1 * q1 + q0 * q0);
      float pr = asin(2 * q2 * q3 + 2 * q0 * q1);
      float rr = atan2(-2 * q1 * q3 + 2 * q0 * q2, q3 * q3 - q2 * q2 - q1 * q1 + q0 + q0);

      y = yr * 180 / pi_numeric;
      p = pr * 180 / pi_numeric;
      r = rr * 180 / pi_numeric;
    
    }
}

double pid(double actual) {
  unsigned long pid_elapsed_time = millis();
  if (millis() - pid_elapsed_time > 45) {
    pid_elapsed_time = millis();
    double currentTime = millis();
    double elapsedTime = currentTime - previousTime;
    double error = setpoint - actual;
    double proportional = error;
    integral += error * elapsedTime;
    double derivative = (error - previous) / elapsedTime;
    previous = error;
    previousTime = currentTime;
    double output = (kp * proportional) + (ki * integral) + (kd * derivative);
    //Serial.print("Pitch: "); Serial.print(ypr[1] * 180/M_PI);
    //Serial.print(" Output: "); Serial.println(output);
    return output;
  }
}

void tvc_update() {
  float x_output = pid((ypr[1] * 180 / M_PI));
  delay(50);
  float y_output = pid((ypr[2] * 180 / M_PI));
  x_servo.write(90 - x_output);
  y_servo.write(90 + y_output);
  /*Serial.print("\t");
    Serial.print(-x_output);
    Serial.print("\t");
    Serial.print(y_output);
    Serial.println("\t");*/
}
