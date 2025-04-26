#include "motor.hpp"
#include <tle94112-ino.hpp>
#include "Arduino_BMI270_BMM150.h"

void setup(){
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

    Serial.println("Starting Motor Dead-Band Test");
}

void loop(){
    // Sweep positive PWM values:
    for (int pwm = 0; pwm <= 100; pwm++){
        Serial.print("PWM = ");
        Serial.println(pwm);
        motor_pwm(1, pwm); // send to motor 1
        motor_pwm(2, pwm); // send to motor 2
        delay(500);  // adjust delay as needed for observation
    }
    
    delay(2000); // pause to observe
    
    // Sweep negative PWM values:
    for (int pwm = 0; pwm >= -100; pwm--){
        Serial.print("PWM = ");
        Serial.println(pwm);
        motor_pwm(1, pwm);
        motor_pwm(2, pwm);
        delay(500);
    }
    
    delay(5000); // pause before repeating the test
}