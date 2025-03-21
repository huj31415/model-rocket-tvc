#include <Arduino.h>
// Servo
#include <Servo.h>
// MPU
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
// SD
#include <SPI.h>
#include <SD.h>
// OTA update
#include <ArduinoOTA.h>
#include <ESP8266WiFi.h>
#include "networkauth.h" // for network ssid and password
// Vectors and quaternions
#include "VectorMath.h"

#define INNER_SERVO_PIN 2
#define OUTER_SERVO_PIN 0
#define SD_CS_PIN 16 //15
#define BUTTON_PIN 9 //16
#define LED_PIN 15


// enum State
// {
//   IDLE,
//   LAUNCH_DETECT,
//   PWR_FLIGHT,
//   COAST
// };

// time increment in sec
double dt;
// loop start time in micros
unsigned long start;
// current time in micros
unsigned long elapsedTime;

unsigned long lastSaveTime = 0;
// SD card file save interval in us
const double saveInterval = 5 * 1e6;


// Servo variables

// Create servos
Servo servoInner;
Servo servoOuter;

// Servo PWM widths (0-180 deg.)
const int MIN_PULSE = 250;
const int MAX_PULSE = 2250;

// inner servo limits
const int S1_MAX = 70;
const int S1_MIN = -70;

// outer servo limits
const int S2_MAX = 70;
const int S2_MIN = -80;

// Motor deflection limits, in rad. (originally +/-25 deg.)
const double deflectionLimit = radians(15);

// Create IMU
Adafruit_MPU6050 mpu;


// State variables

// Position vector
// Vector3d pos = Vector3d::Zero();
// // Velocity vector
// Vector3d vel = Vector3d::Zero();
// // Acceleration vector
// Vector3d localAccel = Vector3d::Zero();
// Vector3d globalAccel = Vector3d::Zero();
// Initial gravity vector
Vector3d gravity = Vector3d::Zero();

// Local angular velocity
Vector3d omega = Vector3d::Zero();
// Euler rotation
Vector3d eulerRot = Vector3d::Zero();
// Local decoupled angular position (pitch and yaw only)
Vector2d decoupledRot = Vector2d::Zero();
// Initial angular velocity error
Vector3d gyroErr = Vector3d::Zero();
// Rotation from body frame to world frame
QuaternionD globalRot = QuaternionD::Identity();
// Initial rotation of the rocket
QuaternionD initialRot = QuaternionD::Identity();

// PID setpoint, set to vertical and const for now
const Vector2d setpointV = Vector2d::Zero();
// const QuaternionD setpointQ = QuaternionD::Identity();

// Sensor outputs
sensors_event_t a, g, temp;

// Data file
File dataFile;

bool useWifi = true;

bool lastState = false;

// PID vars
// Vector2d PIDErr = Vector2d::Zero();
// Vector2d PIDInt = Vector2d::Zero();
// PID gains, derivative filter, and anti-windup constant
double KU = 1, TU = 1, N = 10, Tt = 1.0;
double KP, KI, KD;
Vector2d PIDErr = Vector2d::Zero();
Vector2d PIDOut = Vector2d::Zero();

struct FourBarParams
{
  double a, b, c, d, n, zero;
  int inv;
  double aSqr, bSqr, cSqr, dSqr, ad2, acosLim;
  FourBarParams(double a, double b, double c, double d, double n, double zero, int inv)
      : a(a), b(b), c(c), d(d), n(n), zero(zero), inv(inv),
        aSqr(a * a),
        bSqr(b * b),
        cSqr(c * c),
        dSqr(d * d),
        ad2(a * d * 2.0),
        acosLim(acos(d / a))
  {
  }
};

// 4 bar linkage definitions, in mm

// Inner axis dimensions
const FourBarParams inner = {
    40.0,              // output arm
    27.0,              // connecting arm //27.7
    16.5,              // servo arm //18.5
    29.0,              // fixed arm //30.0
    atan2(21.365, 21), // angle from D to vertical, originally reversed
    -24.263749,        // value to set function to 0 at t=0
    1                  // don't invert angle
};

// Outer axis dimensions
const FourBarParams outer = {
    40.0,              // output arm
    28.8,              // connecting arm
    16.5,              // servo arm
    30.5,              // fixed arm
    atan2(22.3, 20.9), // angle from D to vertical, originally reversed
    -24.475072,        // value to set function to 0 at t=0
    -1                 // invert angle
};

/**
 * Clamps a number between a min and max
 */
double clamp(double x, double low, double high)
{
  return min(max(x, low), high);
}

/**
 * Converts target output angle to required servo angle using 4-bar linkage
 * @param trad Target output angle in rad.
 * @param ax Dimensions and parameters of the 4-bar linkage
 * @returns Servo angle in deg. required for the target angle
 */
double getServoAngleFromTargetAngle(double trad, FourBarParams ax)
{
  trad = -trad - ax.n; // clamp(, -deflectionLimit, deflectionLimit); // -pi/2 + n

  double f = sqrt(ax.aSqr + ax.dSqr - ax.ad2 * cos(trad));
  double as = asin((ax.a * sin(trad)) / f);
  // double a = trad < -ax.acosLim ? (-PI - as) : (trad > ax.acosLim ? (PI - as) : as);
  double a = abs(trad) <= ax.acosLim ? as : copysign(PI, trad) - as;
  double c = acos((ax.bSqr - f * f - ax.cSqr) / (-2 * f * ax.c));
  return ax.zero - degrees(a + c);
}

/**
 * Write both servos relative to 90 with limits
 * @param v Vector with x = inner axis, y = outer axis angle, both in rad.
 */
void actuateServos(Vector2d v)
{
  servoInner.write(85 + clamp((int)(inner.inv * getServoAngleFromTargetAngle(v.x, inner)), S1_MIN, S1_MAX));
  servoOuter.write(90 + clamp((int)(outer.inv * getServoAngleFromTargetAngle(v.y, outer)), S2_MIN, S2_MAX));
}

/**
 * Initialize servos and set angle to 0
 */
void initServos() {
  // init servos
  servoInner.attach(INNER_SERVO_PIN, MIN_PULSE, MAX_PULSE); // servoInner -> pin IO2 //2
  servoOuter.attach(OUTER_SERVO_PIN, MIN_PULSE, MAX_PULSE); // servoOuter -> pin IO0 //0
  // set motor to vertical
  actuateServos(Vector2d::Zero());
}

/**
 * Attempts to initialize OTA updates
 */
void initOTA()
{
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  uint32_t notConnectedCounter = 0;
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(250);
    digitalWrite(LED_PIN, HIGH);
    delay(250);
    digitalWrite(LED_PIN, LOW);

    Serial.println("Wifi connecting...");
    notConnectedCounter++;
    if (notConnectedCounter > 10)
    { // Reset board if not connected after 5s
      Serial.println("Wifi not connected");
      // ESP.restart();
      useWifi = false;
      return;
    }
  }
  Serial.print("Wifi connected, IP address: ");
  Serial.println(WiFi.localIP());
  // Enable OTA
  ArduinoOTA.begin();
  // check for ota updates
  ArduinoOTA.handle();
}

/**
 * Attempts to initialize the MPU6050
 */
void initMPU6050()
{
  // init MPU6050, SCL -> IO5, SDA -> IO4
  while (!mpu.begin())
  {
    Serial.println("Failed to init MPU6050");
    // check for ota updates
    if (useWifi) ArduinoOTA.handle();

    // flash ---.---.
    digitalWrite(LED_PIN, HIGH);
    delay(750);
    digitalWrite(LED_PIN, LOW);
    delay(250);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G); // 2,4,8,16
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);      // 250,500,1000,2000
  mpu.setFilterBandwidth(MPU6050_BAND_10_HZ);   // 260,184,94,44,21,10,5
}

/**
 * Attempts to initialize SD card, open file, and write CSV headers
 */
void initSD()
{
  unsigned int fileNum = 0;

  while (!SD.begin(SD_CS_PIN))
  {
    Serial.println("Failed to initialize SD");
    // check for ota updates
    if (useWifi) ArduinoOTA.handle();
    digitalWrite(LED_PIN, HIGH);
    delay(250);
    digitalWrite(LED_PIN, LOW);
    delay(750);
  }

  // Open and read PID parameters from TVCData/PID.txt
  File pidFile = SD.open("TVCData/PID.txt", FILE_READ);
  if (pidFile) {
    char line[32];  // Buffer to hold each line
    while (pidFile.available()) {
      int len = pidFile.readBytesUntil('\n', line, sizeof(line) - 1);
      line[len] = '\0';  // Null-terminate the string

      // Parse values based on prefix
      if (strncmp(line, "KU=", 3) == 0) {
        KU = atof(line + 3);
      } else if (strncmp(line, "TU=", 3) == 0) {
        TU = atof(line + 3);
      } else if (strncmp(line, "N=", 2) == 0) {
        N = atof(line + 2);
      } else if (strncmp(line, "Tt=", 3) == 0) {
        Tt = atof(line + 3);
      }
    }
    pidFile.close();
    // Update PID gains with Ziegler-Nichols tuning
    KP = 0.6 * KU, KI = 1.2 * KU / TU, KD = 3.0 * KU * TU / 40.0;
    Serial.println("PID parameters loaded.");
  } else {
    Serial.println("Failed to open PID.txt!");
    ESP.restart();
  }

  char fileNameBuf[32];
  sprintf(fileNameBuf, "TVCData/data%d.csv", fileNum);
  while (SD.exists(fileNameBuf))
  {
    fileNum++;
    sprintf(fileNameBuf, "TVCData/data%d.csv", fileNum);
  }
  dataFile = SD.open(fileNameBuf, FILE_WRITE);
  if (!dataFile) ESP.restart();
  Serial.printf("Writing to file %s\n", fileNameBuf);
  dataFile.println("T(us),Qw,Qx,Qy,Qz,ErrX(rad),ErrY(rad),TVCX(rad),TVCY(rad),AccelX(m/s2),AccelY(m/s2),AccelZ(m/s2)");
}

/**
 * Calibrate the sensor with an average reading over (iter/100) seconds,
 * sets initial rotation to the angle of the gravity vector, and
 * sets gravity strength to the magnitude of acceleration
 * @param iter The number of iterations to average over, separated by 10 ms
 */
void calibrateSensors(int iter)
{
  Vector3d accel = Vector3d::Zero();
  Vector3d gyro = Vector3d::Zero();
  // Average over iter iterations
  for (int i = 0; i < iter; i++)
  {
    mpu.getEvent(&a, &g, &temp);
    accel += Vector3d::fromSensorData(a.acceleration);
    gyro += Vector3d::fromSensorData(g.gyro);

    digitalWrite(LED_PIN, lastState ? HIGH : LOW);
    lastState = !lastState;

    delay(10);
  }
  accel /= iter;
  gyro /= iter;

  // Set initialRot and initial globalRot to the angle of the gravity vector, z=roll
  initialRot = globalRot = QuaternionD::fromEuler(
                               atan2(accel.y, accel.z),
                               atan2(-accel.x, sqrt(accel.y * accel.y + accel.z * accel.z)),
                               0)
                               .normalized();

  // Set gravity strength to the magnitude of acceleration
  gravity.z = accel.length();

  // Set gyro error to average value
  gyroErr = gyro;
}

/**
 * Waits until a launch is detected, then continue
 * @param threshold the acceleration threshold for detecting a launch in m/s^2
 * @param freq the frequency to test acceleration in s^-1
 */
void waitForLaunch(double threshold = 1., double freq = 500)
{
  unsigned long iters = 0;
  int period = (int)round(1000. / freq);
  Serial.println("Waiting for launch");
  // wait while non-gravitational acceleration is less than the threshold
  while (abs(Vector3d::fromSensorData(a.acceleration).length() - gravity.length()) < threshold)
  {
    mpu.getEvent(&a, &g, &temp);

    iters++;

    if (iters % (int)(freq / 2) == 0) {
      digitalWrite(LED_PIN, lastState ? HIGH : LOW);
      lastState = !lastState;
    }
    // check for ota updates
    if (useWifi) ArduinoOTA.handle();
    delay(period);
  }
  return;
}

/**
 * Updates the current rotation quaternion of the rocket
 */
void updateState()
{
  // Update sensor readings
  mpu.getEvent(&a, &g, &temp);

  // --- Update rotation ---
  // Get current angular velocity and prepare for quaternion derivative
  omega = (Vector3d::fromSensorData(g.gyro) - gyroErr) * 0.5 * dt;
  // Integrate by the quaternion derivative
  QuaternionD dq = QuaternionD(1, omega.x, omega.y, omega.z).normalized();
  // globalRot * dq -> body to world, dq * globalRot -> world to body
  globalRot = (globalRot * dq).normalized();

  // // --- Update translation ---
  // // Get current acceleration
  // localAccel = Vector3d::fromSensorData(a.acceleration);
  // // Rotate acceleration to world frame
  // globalAccel = globalRot.rotate(localAccel) - gravity;
  // // Integrate position using rotated acceleration
  // vel += globalAccel * dt;
  // pos += vel * dt;
}

/**
 * Decouples pitch and yaw from roll and runs a pair of one-dimensional PIDs
 * @param setpoint Setpoint in pitch and yaw angles
 * @returns Correction angles in pitch and yaw in the same units
 */
Vector2d AxesPID(Vector2d setpoint)
{
  static Vector2d errorI = Vector2d::Zero();

  // Convert to world frame -> local frame Euler angles with roll decoupled
  eulerRot = (QuaternionD::fromEuler(0, 0, (globalRot.conjugate().toEuler()).z).conjugate() * globalRot).toEuler();

  // Decouple rotation axes with z=roll
  decoupledRot = Vector2d(eulerRot.x, eulerRot.y);

  // Run PID
  Vector2d newError = setpoint - decoupledRot;
  Vector2d errorP = newError;
  errorI += newError * dt;
  Vector2d errorD = (newError - PIDErr) / (dt * (1 + N * dt / 2));
  PIDErr = newError;

  Vector2d output = errorP * KP + errorI * KI + errorD * KD;

  // Integral anti-windup
  double magnitude = output.length();
  if (magnitude > deflectionLimit)
  {
    // Calculate saturated output
    Vector2d satOutput = output * (deflectionLimit / magnitude);
    // Back-calculation
    Vector2d windupErr = (satOutput - output) / KI;
    errorI += windupErr * (dt / Tt);
    // Apply saturated output
    output = satOutput;
  }

  return output;
}

/**
 * Closes the file and restarts ESP8266 when the interrupt button is pressed
 */
IRAM_ATTR void reset_ISR() {
  dataFile.close();
  digitalWrite(LED_PIN, LOW);
  delay(100);
  ESP.restart();
}

void setup()
{
  Serial.begin(74880);
  Serial.println();

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);

  // Initialize everything
  initServos();

  initOTA();

  initMPU6050();

  initSD();

  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), reset_ISR, FALLING);


  delay(1000);

  calibrateSensors(200);

  waitForLaunch();

  Serial.println("Loop start");
  digitalWrite(LED_PIN, HIGH);

  elapsedTime = 0;
  start = micros();
}

void loop()
{
  // check for ota updates
  if (useWifi) ArduinoOTA.handle();

  // Update dt
  dt = ((micros() - start) - elapsedTime) / 1.e6;
  elapsedTime = micros() - start;

  updateState();
  PIDOut = AxesPID(setpointV);
  actuateServos(PIDOut);
  // test();

  // Print data to file
  dataFile.printf("%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%3f,%3f,%3f\n",
                  elapsedTime,
                  globalRot.w,
                  globalRot.x,
                  globalRot.y,
                  globalRot.z,
                  PIDErr.x,
                  PIDErr.y,
                  PIDOut.x,
                  PIDOut.y,
                  a.acceleration.x,
                  a.acceleration.y,
                  a.acceleration.z
                );
  if (elapsedTime - lastSaveTime > saveInterval)
  {
    dataFile.flush();
    lastSaveTime = elapsedTime;
  }
}
