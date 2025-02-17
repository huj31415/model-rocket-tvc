#include <Arduino.h>

#include <Servo.h>

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#include "VectorMath.h"

// time increment
double dt;
// loop start time in micros
unsigned long start;
// current time in micros
unsigned long currentTime;

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

// IMU and state variables

// Create IMU
Adafruit_MPU6050 mpu;

// Position vector
Vector3d localPos = Vector3d::Zero();
// Velocity vector
Vector3d localVel = Vector3d::Zero();
// Acceleration vector
Vector3d localAccel = Vector3d::Zero();

// Local angular velocity
Vector3d omega = Vector3d::Zero();
// Local angular position
// Vector3d localRot = Vector3d::Zero();
// Local decoupled angular position
Vector3d decoupledRot = Vector3d::Zero();
// Initial angular velocity error
Vector3d omegaErr = Vector3d::Zero();
// Global rotation
QuaternionD globalRot = QuaternionD::Identity();
// Initial rotation
QuaternionD initialRot = QuaternionD::Identity();

// Sensor outputs
sensors_event_t a, g, temp;

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
 * @param ax dimensions and parameters of the 4-bar linkage
 * @returns Servo angle required for the target angle
 */
double getServoAngle(double t, FourBarParams ax) {
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
void actuateMotor(int val1, int val2) {
  servo1.write(90 + min(max((int)(inner.inv * getServoAngle(val1, inner)), S1_MIN), S1_MAX));
  servo2.write(90 + min(max((int)(outer.inv * getServoAngle(val2, outer)), S2_MIN), S2_MAX));
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
 * Integrates motion twice with semi-implicit Euler's method
 * @param accel Current accleration vector
 * @param vel Reference to velocity vector
 * @param pos Reference to position vector
 * @param dt Timestep
 */
void updatePos(Vector3d accel, Vector3d& vel, Vector3d& pos, double dt) {
  vel += accel * dt;
  pos += vel * dt;
}

/**
 * Integrates rotation
 * @param omega Current angular velocity in x, y, z
 * @param globalRot Reference to current global rotation
 * @param decoupledRot Reference to current roll-decoupled rotation
 */
void updateRot(Vector3d omega, QuaternionD& globalRot, Vector3d& decoupledRot, double dt) {
  // globalRot *= (QuaternionD(0, omega.x, omega.y, omega.z) * dt).normalized();
  globalRot = (initialRot + (initialRot * QuaternionD(0, omega.x, omega.y, omega.z) / 2) * dt).normalized();
  Vector3d eulerRot = globalRot.toEulerAngles();
  decoupledRot.z = cos(eulerRot.x) * eulerRot.z - sin(eulerRot.x) * eulerRot.y;
  decoupledRot.y = cos(eulerRot.x) * eulerRot.y + sin(eulerRot.x) * eulerRot.z;
}

/**
 * Averages acceleration or gyro sensor readings over a specified duration
 * @param ms Time in ms to average over
 * @param type Data type, 'a' -> acceleration, 'g' -> gyro
 * @returns The average value of the sensor, or Vector3d::Zero() if type isn't 'a' or 'g'
 */
Vector3d sensorAverage(unsigned int ms, char type) {
  unsigned int start = millis();
  Vector3d out = Vector3d::Zero();
  for (unsigned int t = start; t < start + ms; t = millis()) {
    mpu.getEvent(&a, &g, &temp);
    sensors_vec_t data = a.acceleration;
    switch (type) {
      case 'a':
        data = a.acceleration;
        break;
      case 'g':
        data = g.gyro;
        break;
    }
    out += Vector3d::fromSensorData(data);
  }
  return out / ms;
}

Vector3d kalmanFilter(sensors_vec_t raw) {
  Vector3d out = Vector3d::fromSensorData(raw);
  return out;
}

void setup() {
  Serial.begin(9600);
  Serial.println("Begin");

  // init servos
  servo1.attach(2, MIN_PULSE, MAX_PULSE);  // servo1 -> pin IO2 //4
  servo2.attach(0, MIN_PULSE, MAX_PULSE);  // servo2 -> pin IO0 //2
  // set motor to vertical
  actuateMotor(0, 0);

  init6050();

  delay(1000);

  omegaErr = sensorAverage(2000, 'g');
  Vector3d initAccel = sensorAverage(2000, 'a');
  initialRot = QuaternionD::fromEulerAngles(0, initAccel.y, initAccel.z);

  start = micros();
  currentTime = start;
}

void loop() {
  // Update dt
  dt = (micros() - currentTime) / 1.e6;
  currentTime = micros() - start;

  // Get sensor readings
  mpu.getEvent(&a, &g, &temp);

  // Update motion
  localAccel = Vector3d::fromSensorData(a.acceleration);
  updatePos(localAccel, localVel, localPos, dt);

  // Update rotation
  omega = Vector3d::fromSensorData(g.gyro);
  updateRot(omega, globalRot, decoupledRot, dt);

  delay(500);
}
