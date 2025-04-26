#include <tle94112-ino.hpp>
#include "Arduino_BMI270_BMM150.h"
#include "motor.hpp"

// Stage 1: P-only tuning:
//   • Set ki = 0 and kd = 0.
//   • Increase kp until you observe a just-perceptible oscillation when the bot is tipped (call this value Ku).
//   • Set kp ≈ 0.6 × Ku (i.e kp) for your initial tuned value.
// Stage 2: Add I-term:
//   • Set ki ≈ 0.1 × (final tuned kp).
//   • Observe if there is any long-term drift (lean creeping) and adjust ki accordingly.
// Stage 3: Add D-term:
//   • Start with kd ≈ 0.01 × (final tuned kp).
//   • Gently tap the robot to check for overshoot. Increase kd until overshoot is minimized, but avoid high-frequency jitter.
// PID parameters
float kp = 30; // Proportional factor 26
float ki = 3; // Integral factor 1.5
float kd = 0.3; // Derivative factor 7.25

// Anti-windup and emergency-stop constants
const float I_MAX = 50.0;
const float EMERGENCY_ANGLE = 30.0;   // degrees
const float EMERGENCY_TIME  = 0.2;    // seconds
float over_tilt_timer = 0.0;

// State variables for PID and filtering
float integral = 0.0;
float previous_error = 0.0; // Previous error for derivative calculation
float filter_alpha = 0.98; // 98% gyro, 2% accel
float filt_gyro_state = 0.0;
float filt_accel_angle = 0.0;
float gyro_bias = 0.0;
float accel_angle_offset = 0.0;
float angle_should = 0; // Target angle
float angle_is = 0; // Current filtered angle
float offset_angle = 0.5;

float offset_motor = 20;

// Timing
unsigned long last_micros = 0;

// Tle94112 object for motor controller
Tle94112Ino controller = Tle94112Ino(3);

// Insert calibration function before setup()
void calibrateSensors() {
    const int samples = 150;
    float gyroSum = 0.0;
    float ax, ay, az;

    Serial.println("Calibrating sensors... Keep robot level and still");
    delay(200); // wait for 200ms before sampling

    for (int i = 0; i < samples; i++) {
        // Wait until fresh sensor data is available
        while (!IMU.accelerationAvailable() || !IMU.gyroscopeAvailable()) {
            delay(1);
        }
        float gx, gy, gz;
        IMU.readGyroscope(gx, gy, gz);
        gyroSum += gx;
        IMU.readAcceleration(ax, ay, az);
        delay(5); // small delay between samples
    }
    gyro_bias = gyroSum / samples;

    // Wait until data is available to calibrate accelerometer offset
    while (!IMU.accelerationAvailable()) {
         delay(1);
    }
    IMU.readAcceleration(ax, ay, az);
    accel_angle_offset = atan2(ay, az) * 180 / PI;
    
    Serial.print("Gyro bias: ");
    Serial.println(gyro_bias);
    Serial.print("Accel angle offset: ");
    Serial.println(accel_angle_offset);
}

float ramOutput(float rawOutput, float dt) {
    static float lastOutput = 0.0;
    static bool firstBurst = true;

    if (firstBurst && abs(rawOutput) > 150) {
        lastOutput = rawOutput;
        firstBurst = false;
        return rawOutput;
    }

    const float RAMP_RATE = 400.0; // PWM
    float maxDelta = RAMP_RATE * dt;
    float delta = rawOutput - lastOutput;

    if (abs(delta) > maxDelta) {
        delta = (delta > 0 ? +1 : -1) * maxDelta;
    } 
    lastOutput += delta;

    if(abs(lastOutput) >255) lastOutput = (delta > 0 ? +1 : -1) * 255;

    Serial.println(lastOutput);

    return lastOutput;
}

void serialPIDInput(){

    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        if (input.startsWith("p=")){
            kp = input.substring(2).toFloat();
        }
        else if (input.startsWith("i=")){
            ki = input.substring(2).toFloat();
        }
        else if (input.startsWith("d=")){
            kd = input.substring(2).toFloat();
        }
        Serial.print("Updated PID: ");
        Serial.print(" kp=");Serial.print(kp);
        Serial.print(" ki=");Serial.print(ki);
        Serial.print(" kd=");Serial.print(kd);
    }
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
    // start CSV logging
    Serial.println("time,angle,output");
    last_micros = micros();
    Serial.println("Serial started");

    // Initialize the IMU sensor
    if (!IMU.begin()) {
        Serial.println("Failed to initialize IMU!");
        while (1); // Halt execution if IMU initialization fails
    }

    // Calibrate sensors once before entering loop()
    calibrateSensors();

    // initialize filter states
    filt_accel_angle = 0.0;
    filt_gyro_state = 0.0;

    // Print IMU sample rate
    Serial.print("Accelerometer sample rate = ");
    Serial.print(IMU.accelerationSampleRate());
    Serial.println(" Hz");
    Serial.println();
    Serial.println("Acceleration in G's");
    Serial.println("X\tY\tZ");
}

void loop() {
    // allow live PID tuning
    serialPIDInput();

    // compute loop delta-time
    unsigned long now = micros();
    float dt = (now - last_micros) * 1e-6;
    last_micros = now;

    // Log loop frequency once every second
    static unsigned long lastRatePrint = 0;
    if ((now - lastRatePrint) >= 1000000UL) {
        Serial.print("Loop Frequency: ");
        Serial.print(1.0/dt, 1);
        Serial.println(" Hz");
        lastRatePrint = now;
    }
    
    // emergency cutoff on large tilt
    if (abs(angle_is) > EMERGENCY_ANGLE) {
      over_tilt_timer += dt;
      if (over_tilt_timer >= EMERGENCY_TIME) {
        motor_pwm(1, 0);
        motor_pwm(2, 0);
        integral = 0;
        return;
      }
    } else {
      over_tilt_timer = 0;
    }

    // Check if acceleration data is available
    if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
        /*
            read the data
        */
        float ax, ay, az;
        IMU.readAcceleration(ax, ay, az); // Read acceleration values
        float gx, gy, gz;
        IMU.readGyroscope(gx, gy, gz);

        // Subtract gyro bias from the x-axis reading and low-pass filter it:
        float corrected_gx = gx - gyro_bias;
        filt_gyro_state = 0.9 * filt_gyro_state + 0.1 * corrected_gx;

        /*
            Calculate the current pitch angle from the accelerometer and subtract offset
        */
        float accel_angle = (atan(ay / az) * 180 / PI) - accel_angle_offset; // Convert to degrees
        filt_accel_angle = 0.95 * filt_accel_angle + 0.05 * accel_angle;
 
        // Complementary filter with the tuned α value for both gyro and accelerometer
        angle_is = filter_alpha * (angle_is + filt_gyro_state * dt) + (1 - filter_alpha) * filt_accel_angle;
        
        // Zero out the angles for noise rejection
        if (abs(angle_is) <= offset_angle) angle_is = 0.0;

        /*
            PID controller calculations
        */
        float error = (angle_should - angle_is);
        integral += error * dt; 
        integral = constrain(integral, -I_MAX, I_MAX);

        // Calculate derivative term
        float derivative = (error - previous_error) / dt;

        // Compute friction feed-forward term based on motor dead-band
        float frictionFF = (error > 0 ? offset_motor : -offset_motor);

        // Compute PID output with friction compensation
        float output = kp * error + ki * integral + kd * derivative + frictionFF;
        previous_error = error; // Update previous error

        // Apply soft-start ramp to the output
        float safeOutput = ramOutput(output, dt);

        // Constrain PID output to the range [-255, 255] with power scaling
        float power_scaling = 0.95;
        safeOutput = constrain(safeOutput, -255 * power_scaling, 255 * power_scaling);
        
        // Adjust motor speed based on the final PID output plus feed-forward
        motor_pwm(1, safeOutput); // Control motor 1
        motor_pwm(2, safeOutput); // Control motor 2

        // CSV output: time(s), angle(°), command
        Serial.print(now * 1e-6, 3);
        Serial.print(",");
        Serial.print(angle_is, 2);
        Serial.print(",");
        Serial.println(safeOutput, 1);
    }
}