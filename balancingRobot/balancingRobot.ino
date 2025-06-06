#include <tle94112-ino.hpp>
#include "Arduino_BMI270_BMM150.h"
#include "motor.hpp"

// PID parameters
float kp = 2.0; // Proportional factor - until the robot just begins to oscillate around upright.
float ki = 0.4; // Integral factor -  until any steady lean (creep) goes away, then stop.
float kd = 0.8; // Derivative factor - until pushes are snappy but not overly jittery.
// Angle
float angle_should = 0.0; // Target angle
float previous_error = 0.0; // Previous error for derivative calculation
float integral = 0.0; // Integral term for PID
float angle_is = 0.0; // Current filtered angle
// Anti-windup limit -> maximum magnitude of the integral term
const float I_MAX = 100.0;   
// timestamp of the previous loop for calculating dt
unsigned long lastMicros;
// Emergency stop parameters for a big tilt
// const float EMERGENCY_ANGLE= 30.0;
// const float EMERGENCY_TIME = 0.2;
// float overTiltTimer = 0;

// Tle94112 object for motor controller
Tle94112Ino controller = Tle94112Ino(3);

float rampOutput(float rawOutput, float dt) {
    static float lastOutput = 0;
    const float RAMP_RATE = 100.0; // units of PWM per second
    float maxDelta = RAMP_RATE * dt;
    float delta    = rawOutput - lastOutput;

    if (abs(delta) > maxDelta) {
      delta = (delta > 0 ? +1 : -1) * maxDelta;
    }
    lastOutput += delta;

    return lastOutput;
  }

void setup() {
    // Initialize pin 5 as output and set it high after a delay
    pinMode(4, OUTPUT);
    delay(100);
    digitalWrite(4, HIGH);
    delay(100);

    // Initialize the motor controller
    controller.begin();

    /*
        test both motors
    */
    // Configure motor 2 (PWM2) for initial state
    controller.configHB(controller.TLE_HB7, controller.TLE_HIGH, controller.TLE_NOPWM);
    (controller.hasError()) controller.clearErrors();
    controller.configHB(controller.TLE_HB8, controller.TLE_HIGH, controller.TLE_NOPWM);
    (controller.hasError()) controller.clearErrors();
    controller.configHB(controller.TLE_HB9, controller.TLE_LOW, controller.TLE_PWM2);
    (controller.hasError()) controller.clearErrors();
    controller.configHB(controller.TLE_HB10, controller.TLE_LOW, controller.TLE_PWM2);
    (controller.hasError()) controller.clearErrors();
    controller.configPWM(controller.TLE_PWM2, controller.TLE_FREQ200HZ, 127);
    (controller.hasError()) controller.clearErrors();

    // Configure motor 1 (PWM1) for initial state
    controller.configHB(controller.TLE_HB1, controller.TLE_HIGH, controller.TLE_NOPWM);
    (controller.hasError()) controller.clearErrors();
    controller.configHB(controller.TLE_HB2, controller.TLE_HIGH, controller.TLE_NOPWM);
    (controller.hasError()) controller.clearErrors();
    controller.configHB(controller.TLE_HB3, controller.TLE_LOW, controller.TLE_PWM1);
    (controller.hasError()) controller.clearErrors();
    controller.configHB(controller.TLE_HB4, controller.TLE_LOW, controller.TLE_PWM1);
    (controller.hasError()) controller.clearErrors();
    controller.configPWM(controller.TLE_PWM1, controller.TLE_FREQ200HZ, 127);
    (controller.hasError()) controller.clearErrors();

    // Clear any errors in the controller
    controller.clearErrors();

    // Initialize serial communication
    Serial.begin(9600);
    while (!Serial); // Wait for serial connection
    Serial.println("Started");

    lastMicros = micros();    // prime the loop timer

    // Initialize the IMU sensor
    if (!IMU.begin()) {
        Serial.println("Failed to initialize IMU!");
        while (1); // Halt execution if IMU initialization fails
    }

    // Print IMU sample rate
    Serial.print("Accelerometer sample rate = "); Serial.print(IMU.accelerationSampleRate());
    Serial.println(" Hz");
    Serial.println();
    Serial.println("Acceleration in G's");
    Serial.println("X\tY\tZ");

    // Print Header for the CSV data stream
    // Serial.println("time,angle,output");

    delay(2000); // Delay for second before starting the loop
}

void loop() {
    // Check if acceleration data is available
    if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
        // calculate the time step dt
        unsigned long now = micros();
        float dt = (now - lastMicros) * 1e-6; // dt in seconds
        lastMicros = now;

        // Emergency cutoff:
        // if (abs(angle)>EMERGENCY_ANGLE) {
        //     overTiltTimer += dt;
        //     if (overTiltTimer>=EMERGENCY_TIME) {
        //         motor_pwm(1,0); motor_pwm(2,0);
        //         integral=0; 
        //         return;
        //     }
        // } else overTiltTimer=0;

        /*
            1) Read acceleration and gyroscope data
        */ 
        float ax, ay, az;
        IMU.readAcceleration(ax, ay, az);   // Read acceleration values
        float rx, ry, rz;   
        IMU.readGyroscope(rx, ry, rz); // Read gyroscope values?

        /* 
            2) Complementary filter to combine accelerometer and gyroscope data
        */ 
        // Calculate the current angle (e.g., pitch) based on acceleration values
        // float angle = atan(ay / az) * 180 / PI;
        float accelAngle = atan( ay/az ) * 180 / PI;
        float gyroRate = rx; // deg/s
        angle_is = 0.90 * (angle_is + gyroRate * dt) + (0.1) * accelAngle; // tweak between 0.90–0.995

        /*
            3) PID controller calculations
        */ 
        float error = angle_should - angle_is;     // Calculate error
        // integral += error;   // Update integral term
        integral += error * dt;     // Update integral term with time step (dt)

        // Constrain the integral term to prevent windup
        // If your motor PWM range is ±255, you might start with I_MAX = 50…200.
        integral = constrain(integral, -I_MAX, +I_MAX);

        // Calculate derivative term
        float derivative = ( error - previous_error ) / dt;
        previous_error = error;     // Update previous error

        // Compute PID output
        float output = kp * error + ki * integral + kd * derivative;

        // Constrain PID output to the range [-255, 255]
        output = constrain(output, -255, 255);

        // Soft-start ramp:
        // float safeOutput = rampOutput(rawOutput, dt);

        /* 
            Adjust motor speed based on PID output
        */ 
        // motor_pwm(1, output); // Control motor 1
        // motor_pwm(2, output); // Control motor 2

        // Soft Start for the motors
        motor_pwm(1, safeOutput);
        motor_pwm(2, safeOutput);

        // Print angle and PID output to the serial monitor
        Serial.print("Angle: ");    Serial.print(angle_is);
        Serial.print(" | Output: ");    Serial.println(output);

        // Optional: Stream CSV data for external logging
        float timeSec = micros()*1e-6;
        Serial.print(timeSec, 3); Serial.print(",");
        Serial.print(angle_is, 2);   Serial.print(",");
        Serial.println(output, 1);

        // Optional delay to slow down data rate
        // delay(10);
    }
}
