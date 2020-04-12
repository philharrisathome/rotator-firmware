/*!
* @file globals.h
*
* It is a file to define all global variables
*
* Licensed under the GPLv3
*
*/

#ifndef LIBRARIES_GLOBALS_H_
#define LIBRARIES_GLOBALS_H_

#include <Arduino.h>
#include <WString.h>

/** Rotator status */
enum _rotator_status {
    idle = 1, moving = 2, pointing = 4, error = 8
};
/** Rotator Errors */
enum _rotator_error {
    no_error = 1, sensor_error = 2, homing_error = 4, motor_error = 8,
    over_temperature = 12, wdt_error = 16
};
/** Rotator Control Modes */
enum _control_mode {
    position = 0, speed = 1
};

struct _control{
    double input;          ///< Motor Position feedback in deg
    double input_prv;      ///< T-1 Motor Position feedback in deg
    double speed;          ///< Motor Rotation speed in deg/s
    double setpoint;       ///< Position set point in deg
    double setpoint_speed; ///< Speed set point in deg/s
    uint16_t load;         ///< Motor Load in mA
    double u;              ///< Control signal range 0-255
    double p, i, d;        ///< Control gains
};

struct _rotator{
    volatile enum _rotator_status rotator_status; ///< Rotator status
    volatile enum _rotator_error rotator_error;   ///< Rotator error
    enum _control_mode control_mode;              ///< Control mode
    bool home_complete_flag;                      ///< Homing flag
    bool autotune_pending_flag;                   ///< Start autotune
    int8_t inside_temperature;                    ///< Inside Temperature
    double park_az, park_el;                      ///< Park position for both axis
    uint8_t fault_az, fault_el;                   ///< Motor drivers fault flag
    bool switch_az, switch_el;                    ///< End-stop vales
};

_control control_az = { .input = 0, .input_prv = 0, .speed=0, .setpoint = 0,
                        .setpoint_speed = 0, .load = 0, .u = 0, .p = 20.0,
                        .i = 5.0, .d = 5.0 };
_control control_el = { .input = 0, .input_prv = 0, .speed=0, .setpoint = 0,
                        .setpoint_speed = 0, .load = 0, .u = 0, 
//                      .p = 2.099, .i = 0.702, .d = 1.567      // PID
//                      .p = 1.399, .i = 0.281, .d = 0.000      // PI
                        .p = 2.099, .i = 0.000, .d = 0.000      // P
                        };
_rotator rotator = { .rotator_status = idle, .rotator_error = no_error,
                     .control_mode = position, .home_complete_flag = false, .autotune_pending_flag = false,
                     .inside_temperature = 0, .park_az = 0, .park_el = 0,
                     .fault_az = LOW, .fault_el = LOW , .switch_az = false,
                     .switch_el = false};

char const* get_rotator_status_str() {
    switch(rotator.rotator_status) {
        case idle: return "IDL";
        case moving: return "MOV";
        case pointing: return "PNT";
        case error: return "ERR";
    }
    return "???";
}

char const* get_rotator_error_str() {
    switch(rotator.rotator_error) {
        case no_error: return "NOE";
        case sensor_error: return "SNS";
        case homing_error: return "HOM";
        case motor_error: return "MOT";
        case over_temperature: return "TMP";
        case wdt_error: return "WDT";
    }
    return "???";
}

char const* get_rotator_mode_str() {
    switch(rotator.control_mode) {
        case position: return "POS";
        case speed: return "SPD";
    }
    return "???";
}

#endif /* LIBRARIES_GLOBALS_H_ */
