#include <tle94112-ino.hpp>
#include "Arduino_BMI270_BMM150.h"
#include "motor.hpp"

// Anti-windup and emergency-stop constants
const float I_MAX = 50.0;
const float EMERGENCY_ANGLE = 30.0;   // degrees
const float EMERGENCY_TIME  = 0.2;    // seconds

// State variables for PID and filtering
float integral = 0.0;
float gyro_angle = 0.0;
float filt_accel_angle = 0.0;
float filt_gyro_state = 0.0;
float over_tilt_timer = 0.0;

// Timing
unsigned long last_micros = 0;

// PID parameters
float kp = 30; // Proportional factor 26
float ki = 10; // Integral factor 1.5
float kd = 7; // Derivative factor 7.25

float angle_should = 0; // Target angle
float previous_error = 0.0; // Previous error for derivative calculation
//float integral = 0.0; // Integral term for PID
//float gyro_angle = 0;
float angle_is = 0;
float offset_angle = 0.5;
//float filt_accel_angle;

float last_time = 0;
float errorAvg = 0;  // Initialize the average
int count = 0;

float offset_motor = 20;

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
    Serial.println("Serial started");
    while (!Serial); // Wait for serial connection
    // start CSV logging
    Serial.println("time,angle,output");
    last_micros = micros();
    Serial.println("Serial started");

    last_time = micros();

    // Initialize the IMU sensor
    if (!IMU.begin()) {
        Serial.println("Failed to initialize IMU!");
        while (1); // Halt execution if IMU initialization fails
    }
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


void loop() {
    // allow live PID tuning
    serialPIDInput();

    // compute loop delta-time
    unsigned long now = micros();
    float dt = (now - last_micros) * 1e-6;
    last_micros = now;

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
        float gyro = 0;
        IMU.readGyroscope(gx, gy, gz);
        gyro = gx;
        // low-pass filter on gyro rate
        filt_gyro_state = 0.9 * filt_gyro_state + 0.1 * gx;

        /*
            Calculate the current angle (e.g., pitch) based on acceleration values
        */
        float accel_angle = atan(ay / az) * 180 / PI + offset_angle; // Convert to degrees
        filt_accel_angle = 0.95 * filt_accel_angle + 0.05 * accel_angle;


        // float dt = (now - last_time) / 1000000;
        // last_time = now;    

 
        // integrate gyro for angle increment
        gyro_angle += filt_gyro_state * dt;
        angle_is = 0.5 * (angle_is + gyro_angle) + 0.5 * filt_accel_angle ;// + offsetAngle;
        
        if (abs(angle_is) <= offset_angle) angle_is = 0.0;

        /*
            PID controller calculations
        */
        float error = (angle_should - angle_is);//(target_angle - angle)(target_angle - angle); // Calculate error
        // accumulate integral with anti-windup
        integral += error * dt;
        integral = constrain(integral, -I_MAX, I_MAX);

        // Calculate derivative term
        float derivative = (error - previous_error) / dt;
        //float derivative = -0.5* filt_gyro + (error - previous_error) / dt;

        // Compute PID output
        float output = kp * error + ki * integral + kd * derivative;
        previous_error = error; // Update previous error

        // apply soft-start ramp
        float safeOutput = ramOutput(output, dt);

        /*
            Pass Output to Motors
        */
        float power_scaling = 0.95;
        // output = output * power_scaling;

        if (safeOutput < offset_motor && safeOutput > 0) safeOutput = 20;
        if (safeOutput >-offset_motor && safeOutput < 0) safeOutput = -20;

        // float softOutput = ramOutput(output, dt);

        // Constrain PID output to the range [-255, 255]
        safeOutput = constrain(safeOutput, -255 * power_scaling, 255 * power_scaling);
        
        // Adjust motor speed based on PID output
        motor_pwm(1, safeOutput); // Control motor 1
        motor_pwm(2, safeOutput); // Control motor 2

        // Print angle and PID output to the serial monitor
        //Serial.print("NOW: ");
        //Serial.println(dt);

        // CSV output: time(s), angle(°), command
        Serial.print(now * 1e-6, 3);
        Serial.print(",");
        Serial.print(angle_is, 2);
        Serial.print(",");
        Serial.println(safeOutput, 1);
        

        // Print the updated average
        // count++;  // Increment count
        // errorAvg = ((errorAvg * (count - 1)) + angle) / count;  // Update average
        // Serial.print("Current Error Average: ");
        // Serial.println(errorAvg);

        // Optional delay to slow down data rate
        //delay(1);
    }
}