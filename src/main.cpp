/*!
 * @file dc_motor_controller.ino
 *
 * This is the documentation for satnogs rotator controller firmware
 * for dc motors configuration. The board (PCB) is placed in
 * <a href="https://gitlab.com/librespacefoundation/satnogs/satnogs-rotator-controller">
 * satnogs-rotator-controller </a> and is for releases:
 * v2.0
 * v2.1
 * v2.2
 * <a href="https://wiki.satnogs.org/SatNOGS_Rotator_Controller"> wiki page </a>
 *
 * @section dependencies Dependencies
 *
 * This firmware depends on <a href="https://github.com/br3ttb/Arduino-PID-Library">
 * Arduino-PID-Library</a> being present on your system. Please make sure you
 * have installed the latest version before using this firmware.
 *
 * @section license License
 *
 * Licensed under the GPLv3.
 *
 */

#undef MOTOR_USING_L298N
#define MOTOR_USING_L9110S

#define REPORT_TIME        0.25 ///< Reporting interval in s
#define SAMPLE_TIME        0.1  ///< Control loop in s
#define MAX_PWM            255  ///< Set max Speed
#define MIN_PWM            25   ///< Set min Speed
#define POSITION_DEADZONE  1    ///< Control dead zone (degrees)
#define OVER_TEMP          60   ///< Over temperature limit
#define ENC_RATIO          1    ///< Encoder gear ratio
#define MIN_AZI_ANGLE      0    ///< Minimum angle of azimuth
#define MAX_AZI_ANGLE      360  ///< Maximum angle of azimuth
#define MIN_ELE_ANGLE      0    ///< Minimum angle of elevation
#define MAX_ELE_ANGLE      180  ///< Maximum angle of elevation
#define DEFAULT_HOME_STATE LOW  ///< Change to LOW according to Home sensor
#define HOME_SPEED         100  ///< Set speed to find home

#include <PID_v1.h>
#include <PID_AutoTune_v0.h>

#include "globals.h"
#include "easycomm.h"
#include "Motor.h"
#include "rotator_pins.h"
#include "endstop.h"
#include "watchdog.h"
#include "TemperatureSensor.h"
#include "MotorEncoder.h"

uint32_t t_run = 0; // run time of uC (ms)
uint32_t t_report = 0; // report time of uC (ms)
easycomm comm;
PID pid_az(&control_az.input, &control_az.u, &control_az.setpoint, control_az.p,
           control_az.i, control_az.d, P_ON_E, DIRECT);
PID pid_el(&control_el.input, &control_el.u, &control_el.setpoint, control_el.p,
           control_el.i, control_el.d, P_ON_E, DIRECT);
Motor motor_az(AZIMUTH_DRIVE_IN1, AZIMUTH_DRIVE_EN);
Motor motor_el(ELEVATION_DRIVE_IN1, ELEVATION_DRIVE_EN);
endstop switch_az(AZIMUTH_HOME, DEFAULT_HOME_STATE), switch_el(ELEVATION_HOME, DEFAULT_HOME_STATE);
TemperatureSensor temp_sensor;
MotorEncoder encoder_az(AZIMUTH_PHASE_A, AZIMUTH_PHASE_B), encoder_el(ELEVATION_PHASE_A, ELEVATION_PHASE_B);
wdt_timer wdt;
char buffer[60];

enum _rotator_error homing();
void autotune();

void setup() {

    // Initialise serial port
    Serial.begin(115200);
    Serial.println("Started...");

    // Homing switch
    switch_az.init();
    switch_el.init();

    // Serial Communication
    comm.easycomm_init();

    // Initialize DC motors
    motor_az.stop();
    motor_el.stop();

    // Initialize rotary encoders
    encoder_az.set_gear_ratio(ENC_RATIO);
    encoder_el.set_gear_ratio(ENC_RATIO);

    // Initialize control parameters
    pid_az.SetSampleTime(SAMPLE_TIME);
    pid_az.SetOutputLimits(-MAX_PWM, MAX_PWM );
    pid_az.SetMode(AUTOMATIC);
    pid_el.SetSampleTime(SAMPLE_TIME);
    pid_el.SetOutputLimits(-MAX_PWM, MAX_PWM);
    pid_el.SetMode(AUTOMATIC);

    // Initialise temperature sensor
    temp_sensor.init();
    sprintf(buffer, "Temperature is %d deg", temp_sensor.get_temp());
    Serial.println(buffer);

    // Initialize WDT
    wdt.watchdog_init();

    delay(1000);
}

void loop() {

    // Update WDT
    wdt.watchdog_reset();

    // Get end stop status
    rotator.switch_az = switch_az.get_state();
    rotator.switch_el = switch_el.get_state();

    // Run easycomm implementation
    comm.easycomm_proc();

    // Get Motor driver status
    rotator.fault_az = motor_az.get_fault();
    rotator.fault_el = motor_el.get_fault();
    if (rotator.fault_az || rotator.fault_el) {
        rotator.rotator_status = error;
        rotator.rotator_error = motor_error;
    }

    // Get inside Temperature
    temp_sensor.wake_up();
    rotator.inside_temperature = temp_sensor.get_temp();
    temp_sensor.sleep();
    if (rotator.inside_temperature > OVER_TEMP) {
        rotator.rotator_status = error;
        rotator.rotator_error = over_temperature;
    }

    // Get position of both axis
    encoder_az.get_pos(&control_az.input);
    encoder_el.get_pos(&control_el.input);

    // Check rotator status
    if (rotator.rotator_status != error) {
        if (rotator.home_complete_flag == false) {
            // Check home flag
            rotator.control_mode = position;
            // Homing
            Serial.println("Homing...");
            rotator.rotator_error = homing();
            if (rotator.rotator_error == no_error) {
                // No error
                rotator.rotator_status = idle;
                rotator.home_complete_flag = true;
            } else {
                // Error
                rotator.rotator_status = error;
                rotator.rotator_error = homing_error;
            }
        } else if (rotator.autotune_pending_flag) {
            if (rotator.rotator_status == idle) {
                Serial.println("Tuning...");
                autotune();
                rotator.autotune_pending_flag = false;
                rotator.home_complete_flag = false;
            }
        } else {
            // Control Loop
            if ((millis() - t_run) >= SAMPLE_TIME * 1000) {
                // Update control gains
                pid_az.SetTunings(control_az.p, control_az.i, control_az.d);
                pid_el.SetTunings(control_el.p, control_el.i, control_el.d);
                if (rotator.control_mode ==  speed) {
                    control_az.setpoint += control_az.setpoint_speed * SAMPLE_TIME;
                    control_el.setpoint += control_el.setpoint_speed * SAMPLE_TIME;
                    rotator.rotator_status = moving;
                } else {
                    rotator.rotator_status = pointing;
                }
                // Move azimuth and elevation motors
                pid_az.Compute();
                motor_az.move(control_az.u); 
                pid_el.Compute();
                motor_el.move(control_el.u); 
                // Calculate the speeds of both axis
                control_az.speed = (control_az.input - control_az.input_prv) / SAMPLE_TIME;
                control_az.input_prv = control_az.input;
                control_el.speed = (control_el.input - control_el.input_prv) / SAMPLE_TIME;
                control_el.input_prv = control_el.input;
                // Update the run time
                t_run = millis();
                // Idle rotator, dead-band
                if ((abs(control_az.setpoint - control_az.input) <= POSITION_DEADZONE || (control_az.speed == 0)) &&
                    (abs(control_el.setpoint - control_el.input) <= POSITION_DEADZONE || (control_el.speed == 0))) {
                    rotator.rotator_status = idle;
                }
                // Show activity
                digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
            }
            if ((millis() - t_report) >= (REPORT_TIME * 1000)) {
                // Show status
                sprintf(buffer, "[%s|%s|%s] ", get_rotator_status_str(), get_rotator_mode_str(), get_rotator_error_str()); Serial.print(buffer);
                sprintf(buffer, "[E=%8ld/%8ld/%8ld] ", long(control_el.input*100), long(control_el.setpoint*100), long(control_el.u*100)); Serial.print(buffer);
                sprintf(buffer, "[A=%8ld/%8ld/%8ld]", long(control_az.input*100), long(control_az.setpoint*100), long(control_az.u*100)); Serial.print(buffer);
                sprintf(buffer, "(E=%6d/%6d/%6d) ", int(control_el.p*100), int(control_el.i*100), int(control_el.d*100)); Serial.print(buffer);
                sprintf(buffer, "(A=%6d/%6d/%6d)", int(control_az.p*100), int(control_az.i*100), int(control_az.d*100)); Serial.println(buffer);
                // Update the run time
                t_report = millis();
            }
        }
    } else {
        // Error handler, stop motors and disable the motor driver
        motor_az.stop();
        motor_el.stop();
        if (rotator.rotator_error != homing_error) {
            // Reset error according to error value
            rotator.rotator_error = no_error;
            rotator.rotator_status = idle;
        }
    }
}

/**************************************************************************/
/*!
    @brief    Move both axis with one direction in order to find home position,
              end-stop switches
    @return   _rotator_error
*/
/**************************************************************************/
enum _rotator_error homing() {
    
    Serial.println("Homing started...");

    bool isHome_az = false;
    bool isHome_el = false;

    // Reses position
    encoder_az.set_zero();
    encoder_el.set_zero();

    // Move motors with ~constant speed
    motor_az.move(-HOME_SPEED);
    motor_el.move(-HOME_SPEED);

    // Homing loop
    while (isHome_az == false || isHome_el == false) {
        // Update WDT
        wdt.watchdog_reset();
        if (switch_az.get_state() == true && !isHome_az) {
            // Find azimuth home
            motor_az.stop();
            isHome_az = true;
        }
        if (switch_el.get_state() == true && !isHome_el) {
            // Find elevation home
            motor_el.stop();
            isHome_el = true;
        }
        // Get current position
        encoder_az.get_pos(&control_az.input);
        encoder_el.get_pos(&control_el.input);
        // Check if the rotator goes out of limits or something goes wrong (in
        // mechanical)
        if ((abs(control_az.input) > MAX_AZI_ANGLE && !isHome_az)
         || (abs(control_el.input) > MAX_ELE_ANGLE && !isHome_el)) {
            Serial.println("Homing failed.");
            return homing_error;
        }
    }

    // Set the home position and reset all critical control variables
    encoder_az.init_zero();
    encoder_az.set_zero();
    control_az.setpoint = 0;
    encoder_el.init_zero();
    encoder_el.set_zero();
    control_el.setpoint = 0;

    Serial.println("Homing successful.");

    return no_error;
}

/**************************************************************************/
/*!
    @brief    Auto-tune the PID loops for the two axes.
    @return   Nothing.
*/
/**************************************************************************/

void autotune() {

    double input=0, output=0;
    PID_ATune aTune(&input, &output);

    double const aTuneStartValue=0;       // In output units (PWM)
    double const aTuneStep=128;           // In output units (PWM)
    double const aTuneNoise=45;           // In input units (degrees)
    unsigned int const aTuneLookBack=20;  // seconds -> lookback = 80 samples, sampleTime = 250 ms

    //Set the output to the desired starting frequency.
    output=aTuneStartValue;
    aTune.SetNoiseBand(aTuneNoise);
    aTune.SetOutputStep(aTuneStep);
    aTune.SetLookbackSec((int)aTuneLookBack);

    unsigned long startTime = millis();
    unsigned long serialTime = startTime;
    unsigned long const TIMEOUT(5 * 60 * 1000L);
    while ((millis() - startTime) < TIMEOUT) {

        // Update WDT
        wdt.watchdog_reset();

        //pull the input in from the real world
        encoder_el.get_pos(&input);
        
        if (aTune.Runtime())
        {
            //we're done, set the tuning parameters
            sprintf(buffer, "%s:(%6d/%6d/%6d)", (aTune.GetControlType() == 0) ? "PI" : "PID", int(aTune.GetKp()*100), int(aTune.GetKi()*100), int(aTune.GetKd()*100)); Serial.println(buffer);
            return;
        }
        
        if (Serial.available())
            break;

        motor_el.move(output); 
        
        //send-receive with processing if it's time
        if((millis()-serialTime) >= 100)
        {
            sprintf(buffer, "%2d:[%8ld/%8ld/%8ld]", aTune.GetPeakCount(),long(aTune.GetSetpoint()*100), long(input*100), long(output*100)); Serial.println(buffer);
            serialTime=millis();

            // Show activity
            digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        }

        delay(20);
    }

    Serial.println("Autotune failed.");
}

