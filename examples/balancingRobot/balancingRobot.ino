#include <tle94112-ino.hpp>
#include "Arduino_BMI270_BMM150.h"
#include "motor.hpp"

// PID parameters
float kp = 2.0; // Proportional factor - until the robot just begins to oscillate around upright.
float ki = 0.4; // Integral factor -  until any steady lean (creep) goes away, then stop.
float kd = 0.8; // Derivative factor - until pushes are snappy but not overly jittery.
// Angle
float target_angle = 0.0; // Target angle
float previous_error = 0.0; // Previous error for derivative calculation
float integral = 0.0; // Integral term for PID
float angle = 0.0; // Current filtered angle
// Anti-windup limit -> maximum magnitude of the integral term
const float I_MAX = 100.0;   
// timestamp of the previous loop for calculating dt
unsigned long lastMicros;

// Tle94112 object for motor controller
Tle94112Ino controller = Tle94112Ino(3);

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
    lastMicros = micros();    // prime the loop timer
    Serial.println("Started");

    // Initialize the IMU sensor
    if (!IMU.begin()) {
        Serial.println("Failed to initialize IMU!");
        while (1); // Halt execution if IMU initialization fails
    }

    // Print IMU sample rate
    Serial.print("Accelerometer sample rate = ");
    Serial.print(IMU.accelerationSampleRate());
    Serial.println(" Hz");
    Serial.println();
    Serial.println("Acceleration in G's");
    Serial.println("X\tY\tZ");

    delay(5000); // Delay for 5 second before starting the loop
}

void loop() {
    // Check if acceleration data is available
    if (IMU.accelerationAvailable()) {
        // calculate the time step dt
        unsigned long now = micros();
        float dt = (now - lastMicros) * 1e-6; // dt in seconds
        lastMicros = now;

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
        angle = 0.98 * (angle + gyroRate * dt) + (0.02) * accelAngle; // tweak between 0.90–0.995

        /*
            3) PID controller calculations
        */ 
        float error = target_angle - angle;     // Calculate error
        // integral += error;   // Update integral term
        integral += error * dt;     // Update integral term with time step

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

        /* 
            Adjust motor speed based on PID output
        */ 
        motor_pwm(1, output); // Control motor 1
        motor_pwm(2, output); // Control motor 2

        // Print angle and PID output to the serial monitor
        Serial.print("Angle: ");    Serial.print(angle);
        Serial.print(" | Output: ");    Serial.println(output);

        // Optional delay to slow down data rate
        delay(10);
    }
}
