#include <tle94112-ino.hpp>
#include "Arduino_BMI270_BMM150.h"
#include "motor.hpp"

// PID parameters
float kp = 30; // Proportional factor
float ki = 0.0; // Integral factor
float kd = 1; // Derivative factor
float target_angle = 0.0; // Target angle
float previous_error = 0.0; // Previous error for derivative calculation
float integral = 0.0; // Integral term for PID

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
    controller.configHB(controller.TLE_HB8, controller.TLE_HIGH, controller.TLE_NOPWM);
    controller.configHB(controller.TLE_HB9, controller.TLE_LOW, controller.TLE_PWM2);
    controller.configHB(controller.TLE_HB10, controller.TLE_LOW, controller.TLE_PWM2);
    controller.configPWM(controller.TLE_PWM2, controller.TLE_FREQ200HZ, 127);

    // Configure motor 1 (PWM1) for initial state
    controller.configHB(controller.TLE_HB1, controller.TLE_HIGH, controller.TLE_NOPWM);
    controller.configHB(controller.TLE_HB2, controller.TLE_HIGH, controller.TLE_NOPWM);
    controller.configHB(controller.TLE_HB3, controller.TLE_LOW, controller.TLE_PWM1);
    controller.configHB(controller.TLE_HB4, controller.TLE_LOW, controller.TLE_PWM1);
    controller.configPWM(controller.TLE_PWM1, controller.TLE_FREQ200HZ, 127);

    // Clear any errors in the controller
    controller.clearErrors();

    // Initialize serial communication
    Serial.begin(9600);
    while (!Serial); // Wait for serial connection
    Serial.println("Serial started");

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
}

void loop() {
    // Check if acceleration data is available
    if (IMU.accelerationAvailable()) {

        float ax, ay, az;
        IMU.readAcceleration(ax, ay, az); // Read acceleration values


        float gyro;
        if (IMU.gyroscopeAvailable()){
            float gx, gy, gz;
            IMU.readGyroscope(gx, gy, gz);
            gyro = gx;
        }
        Serial.println(gyro);

        // Calculate the current angle (e.g., pitch) based on acceleration values
        float accel_angle = ay / az;
        accel_angle = atan(accel_angle) * 180 / PI; // Convert to degrees
        
        float angle = 0;
        unsigned long now = millis();
        unsigned long last_time = 0;
        float dt = 0.01;
        dt = (now-last_time)/1000;
        last_time = now;
        //angle = (0.98*(angle + gyro*dt* 180 / PI)+0.02*accel_angle);
        angle = accel_angle;


        // PID controller calculations
        float error = (target_angle - angle); //*(target_angle - angle)*(target_angle - angle); // Calculate error
        integral += error * dt; // Update integral term

        // Constrain the integral term to prevent windup
        integral = constrain(integral, -25, 25);

        // Calculate derivative term
        float derivative = error - previous_error;

        // Compute PID output
        float output = kp * error + ki * integral + kd * derivative;
        previous_error = error; // Update previous error

        // Constrain PID output to the range [-255, 255]
        output = constrain(output, -255, 255);

        // Adjust motor speed based on PID output
        motor_pwm(1, output); // Control motor 1
        motor_pwm(2, output); // Control motor 2

        // Print angle and PID output to the serial monitor
        Serial.print("Angle: ");
        Serial.print(angle);
        Serial.print(" | Output: ");
        Serial.println(output);

        // Optional delay to slow down data rate
        delay(10);
    }
}