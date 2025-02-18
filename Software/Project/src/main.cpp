#include <Arduino.h>
// Servo
#include <Servo.h>
// MPU
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
// Vectors and quaternions
#include "VectorMath.h"

// time increment in sec
double dt;
// loop start time in micros
unsigned long start;
// current time in micros
unsigned long elapsedTime;

// Servo variables

// Create servos
Servo servo1;
Servo servo2;

// Servo PWM widths (0-180 deg.)
const int MIN_PULSE = 250;
const int MAX_PULSE = 2250;

// inner servo limits
const int S1_MAX = 70;
const int S1_MIN = -70;

// outer servo limits
const int S2_MAX = 70;
const int S2_MIN = -80;

// Motor deflection limits, in deg.
const int minDef = -25;
const int maxDef = 25;

// Create IMU
Adafruit_MPU6050 mpu;

// State variables

// Position vector
Vector3d pos = Vector3d::Zero();
// Velocity vector
Vector3d vel = Vector3d::Zero();
// Acceleration vector
Vector3d localAccel = Vector3d::Zero();
Vector3d globalAccel = Vector3d::Zero();
// Initial gravity vector
Vector3d gravity = Vector3d::Zero();

// Local angular velocity
Vector3d omega = Vector3d::Zero();
// Euler rotation
Vector3d eulerRot = Vector3d::Zero();
// Local decoupled angular position
Vector3d decoupledRot = Vector3d::Zero();
// Initial angular velocity error
Vector3d omegaErr = Vector3d::Zero();
// Rotation from body frame to world frame
QuaternionD globalRot = QuaternionD::Identity();
// Initial rotation of the rocket
QuaternionD initialRot = QuaternionD::Identity();

// Sensor outputs
sensors_event_t a, g, temp;

// PID vars
Vector2d PIDErr = Vector2d::Zero();
Vector2d PIDInt = Vector2d::Zero();
double KP = 0, KI = 0, KD = 0;

struct FourBarParams {
  double a, b, c, d, n, zero;
  int inv;
  double aSqr, bSqr, cSqr, dSqr, ad2, acosLim;
  FourBarParams(double a, double b, double c, double d, double n, double zero, double inv)
    : a(a), b(b), c(c), d(d), n(n), zero(zero), inv(inv),
      aSqr(a * a),
      bSqr(b * b),
      cSqr(c * c),
      dSqr(d * d),
      ad2(a * d * 2.0),
      acosLim(acos(d / a))
  {}
};

// 4 bar linkage definitions, in mm

// Inner axis dimensions
const FourBarParams inner = FourBarParams(
  40.0,               // output arm
  27.0,               // connecting arm //27.7
  16.5,               // servo arm //18.5
  29.0,               // fixed arm //30.0
  atan2(21, 21.365),  // angle from D to horizontal
  -24.263749,         // value to set function to 0 at t=0
  1                   // don't invert angle
);
// Outer axis dimensions
const FourBarParams outer = FourBarParams(
  40.0,               // output arm
  28.8,               // connecting arm
  16.5,               // servo arm
  30.5,               // fixed arm
  atan2(20.9, 22.3),  // angle from D to horizontal 20.9 22.3
  -24.475072,         // value to set function to 0 at t=0
  -1                  // invert angle
);


/**
 * Converts target output angle to required servo angle using 4-bar linkage
 * @param t Target output angle
 * @param ax Dimensions and parameters of the 4-bar linkage
 * @returns Servo angle required for the target angle
 */
double getServoAngleFromTargetAngle(double t, FourBarParams ax) {
  t = -t - 90.;// + ax.n; // if n is in degrees
  double trad = radians(t + ax.n); // if n is in rad
  double f = sqrt(ax.aSqr + ax.dSqr - ax.ad2 * cos(trad));
  double as = asin((ax.a * sin(trad)) / f);
  double a = trad < -ax.acosLim ? (-PI - as) : (trad > ax.acosLim ? (PI - as) : as);
  double c = acos((ax.bSqr - f * f - ax.cSqr) / (-2 * f * ax.c));
  return ax.zero - degrees(a + c);
}

/**
 * Write both servos relative to 90 with limits
 * @param val1 Inner axis target angle
 * @param val2 Outer axis target angle
 */
void actuateServos(int val1, int val2) {
  servo1.write(90 + min(max((int)(inner.inv * getServoAngleFromTargetAngle(val1, inner)), S1_MIN), S1_MAX));
  servo2.write(90 + min(max((int)(outer.inv * getServoAngleFromTargetAngle(val2, outer)), S2_MIN), S2_MAX));
}

/**
 * Attempts to initialize the MPU6050 and waits if it fails
 */
void init6050() {
  // init MPU6050, SCL -> IO5, SDA -> IO4
  while (!mpu.begin()) {
    Serial.println("Failed to init MPU6050");
    delay(1000);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G); // 2,4,8,16
  mpu.setGyroRange(MPU6050_RANGE_500_DEG); // 250,500,1000,2000
  mpu.setFilterBandwidth(MPU6050_BAND_260_HZ); // 260,184,94,44,21,10,5
}

/**
 * Calibrate the sensor with an average reading over (iter/100) seconds,
 * sets initial rotation to the angle of the gravity vector, and
 * sets gravity strength to the magnitude of acceleration
 * @param iter The number of iterations to average over, separated by 10 ms
 */
void calibrateSensors(int iter) {
  Vector3d accel = Vector3d::Zero();
  Vector3d gyro = Vector3d::Zero();
  // Average over iter iterations
  for (int i = 0; i < iter; i++) {
    mpu.getEvent(&a, &g, &temp);
    accel += Vector3d::fromSensorData(a.acceleration);
    gyro += Vector3d::fromSensorData(g.gyro);
    delay(10);
  }
  accel /= iter;
  gyro /= iter;

  // Set initialRot to the angle of the gravity vector
  initialRot = QuaternionD::fromEulerAngles(
    atan2(accel.y, accel.z),
    atan2(-accel.x, sqrt(accel.y * accel.y + accel.z * accel.z)),
    0
  );

  // Set gravity strength to the magnitude of acceleration
  gravity.z = accel.length();
}

/**
 * Updates the current physical state of the rocket
 */
void updateState() {
  // Update sensor readings
  mpu.getEvent(&a, &g, &temp);

  // --- Update rotation ---
  // Get current angular velocity and prepare for quaternion derivative
  omega = Vector3d::fromSensorData(g.gyro) * 0.5 * dt;
  // Integrate by the quaternion derivative
  QuaternionD dq = QuaternionD(1, omega.x, omega.y, omega.z).normalized();
  globalRot = (globalRot * dq).normalized(); // globalRot * dq -> body to world, dq * globalRot -> world to body
  // Convert to world frame -> local frame Euler angles
  eulerRot = globalRot.inverse().toEulerAngles();
  // --- Decouple rotation axes ---
  // decoupledRot.z = cos(eulerRot.x) * eulerRot.z - sin(eulerRot.x) * eulerRot.y;
  // decoupledRot.y = cos(eulerRot.x) * eulerRot.y + sin(eulerRot.x) * eulerRot.z;
  decoupledRot = (QuaternionD::fromEulerAngles(eulerRot.x, 0, 0).conjugate() * globalRot).toEulerAngles();
  decoupledRot.x = eulerRot.x;
  

  // --- Update translation ---
  // Get current acceleration
  localAccel = Vector3d::fromSensorData(a.acceleration);
  // Rotate acceleration to world frame
  globalAccel = globalRot.rotate(localAccel) - gravity;
  // Integrate position using rotated acceleration
  vel += globalAccel * dt;
  pos += vel * dt;
}

/**
 * Pair of one-dimensional PIDs
 * @param setpoint Setpoint in pitch and yaw angles
 * @param position Current position in pitch and yaw angles
 * @returns Correction angle in pitch and yaw in the same units
 */
Vector2d PID1d(Vector2d setpoint, Vector2d position) {
  static Vector2d newError = setpoint - position;
  Vector2d p = newError * KP;
  PIDInt += newError * KI * dt;
  Vector2d d = (newError - PIDErr) * KD / dt;
  PIDErr = newError;
  return p + PIDInt + d;
}

void printQuaternion(QuaternionD q) {
  Serial.printf("Quat(%.2f, %.2f, %.2f, %.2f)\n", q.w, q.x, q.y, q.z);
}
void printVector(Vector3d v) {
  Serial.printf("Vec3(%.2f, %.2f, %.2f)\n", v.x, v.y, v.z);
}

void setup() {
  Serial.begin(9600);

  // init servos
  servo1.attach(2, MIN_PULSE, MAX_PULSE);  // servo1 -> pin IO2 //4
  servo2.attach(0, MIN_PULSE, MAX_PULSE);  // servo2 -> pin IO0 //2
  // set motor to vertical
  actuateServos(0, 0);

  init6050();

  delay(1000);

  calibrateSensors(200);

  Serial.println("Loop start");
  start = micros();
  elapsedTime = 0;
}

unsigned long lastTime = 0;

void loop() {
  // Update dt
  dt = ((micros() - start) - elapsedTime) / 1.e6;
  elapsedTime = micros() - start;

  updateState();


  if (elapsedTime - lastTime >= 1e6) {
    lastTime = elapsedTime;
    Serial.println(dt);
    Serial.print("pos: ");
    printVector(pos);
    Serial.print("Vel: ");
    printVector(vel);
    Serial.print("Accel: ");
    printVector(localAccel);
    Serial.print("Ang vel: ");
    printVector(omega);
    Serial.print("Global rot: ");
    printQuaternion(globalRot);
    Serial.print("Decoupled rot: ");
    printVector(decoupledRot);
  }
  // if (elapsedTime / 5e6 > 1) delay(10000);

  // delay(500);
}
