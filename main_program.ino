#include <Wire.h>
#include <Servo.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>

// I2Cdev and MPU6050 setup
MPU6050 mpu;
Servo servoRoll, servoPitch;

// PID constants
float Kp = 1.15, Ki = 0.1, Kd = 0.05;
float setPointRoll = 0;  // Desired Roll angle (degrees)
float setPointPitch = 0; // Desired Pitch angle (degrees)
float previousErrorRoll = 0, previousErrorPitch = 0;
float integralRoll = 0, integralPitch = 0;

// Pin definitions
const int servoRollPin = 9;
const int servoPitchPin = 10;

void setup() {
  Wire.begin();
  Serial.begin(9600);

  // Initialize MPU6050 with I2Cdev
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while (1);
  }

  // Initialize the MPU6050 with the default settings
  mpu.initialize();

  // Attach servos
  servoRoll.attach(servoRollPin);
  servoPitch.attach(servoPitchPin);

  Serial.println("System ready!");
}

void loop() {
  // Read accelerometer and gyroscope data
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Convert raw accelerometer data to angle (in degrees)
  float angleRoll = atan2(ay, az) * 180.0 / PI;  // Convert to degrees
  float anglePitch = atan2(ax, az) * 180.0 / PI; // Convert to degrees

  // Calculate PID for Roll
  float errorRoll = setPointRoll - angleRoll;
  integralRoll += errorRoll;
  integralRoll = constrain(integralRoll, -100, 100); // Limit integral
  float derivativeRoll = errorRoll - previousErrorRoll;
  float outputRoll = Kp * errorRoll + Ki * integralRoll + Kd * derivativeRoll;
  previousErrorRoll = errorRoll;

  // Calculate PID for Pitch
  float errorPitch = setPointPitch - anglePitch;
  integralPitch += errorPitch;
  integralPitch = constrain(integralPitch, -100, 100); // Limit integral
  float derivativePitch = errorPitch - previousErrorPitch;
  float outputPitch = Kp * errorPitch + Ki * integralPitch + Kd * derivativePitch;
  previousErrorPitch = errorPitch;

  // Control servos
  servoRoll.write(constrain(outputRoll * 1.5 + 90, 0, 180)); // Center at 90 degrees
  servoPitch.write(constrain(outputPitch * 1.5 + 90, 0, 180));

  // Debugging output
  Serial.print("SetPointPitch,");
  Serial.print(setPointPitch);
  Serial.print(",AnglePitch,");
  Serial.print(anglePitch);
  Serial.print(",OutputPitch,");
  Serial.print(outputPitch);
  Serial.print(",SetPointRoll,");
  Serial.print(setPointRoll);
  Serial.print(",AngleRoll,");
  Serial.print(angleRoll);
  Serial.print(",OutputRoll,");
  Serial.println(outputRoll);

  delay(10); // Delay for stability
}
