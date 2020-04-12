/*!
* @file easycomm.h
*
* It is a driver for easycomm 3 protocol as referred, in Hamlib.
*
* Licensed under the GPLv3
*
*/

#ifndef LIBRARIES_EASYCOMM_H_
#define LIBRARIES_EASYCOMM_H_

#include "globals.h"
#include <avr/wdt.h>

/**************************************************************************/
/*!
    @brief    Class that functions for easycomm 3 implementation
*/
/**************************************************************************/
class easycomm {
public:

    /**************************************************************************/
    /*!
        @brief    Initialize the Serial bus
    */
    /**************************************************************************/
    void easycomm_init() {
    }

    /**************************************************************************/
    /*!
        @brief    Get the commands from Serial and response to the client
    */
    /**************************************************************************/
    void easycomm_proc() {
        String str1, str2, str3, str4, str5, str6;

        // Read from serial
        while (Serial.available() > 0) {
            char incomingByte = toupper(Serial.read());
            // Read new data, '\n' means new pacakage
            if (incomingByte == '\n' || incomingByte == '\r') {
                buffer[BufferCnt] = 0;

                char *dataPtr = buffer;
                if (dataPtr[0] == 'A' && dataPtr[1] == 'Z') { 
                    if (dataPtr[2] == ' ' && dataPtr[3] == 'E' && dataPtr[4] == 'L') {
                        // Get the current absolute position in deg ("AZ EL" -> "AZf ELf")
                        str1 = String("AZ");
                        str2 = String(control_az.input, 1);
                        str3 = String(" EL");
                        str4 = String(control_el.input, 1);
                        str5 = String("\n");
                        Serial.print(str1 + str2 + str3 + str4 + str5);
                    } else {
                        // Set the absolute position in deg for azimuth and (possibly) elevation ("AZf[ ELf]" -> none)
                        rotator.control_mode = position;
                        char* state(0);
                        dataPtr = strtok_r(dataPtr, " ", &state);   // TODO: Isn't strtok_r() being used incorrectly?
                        parse_field(dataPtr + 2, control_az.setpoint);
                        dataPtr = strtok_r(dataPtr, " ", &state);
                        if (dataPtr[0] == 'E' && dataPtr[1] == 'L') {
                            parse_field(dataPtr + 2, control_el.setpoint);
                        }
                    }
                } else if (dataPtr[0] == 'E' && dataPtr[1] == 'L') {
                    // Set the absolute position in deg for elevation ("ELf" -> none)
                    rotator.control_mode = position;
                    parse_field(dataPtr + 2, control_el.setpoint);
                } else if (dataPtr[0] == 'V' && dataPtr[1] == 'U') {
                    // Elevation increase speed in mdeg/s ("VUf" -> none)
                    rotator.control_mode = speed;
                    if (parse_field(dataPtr + 2, control_el.setpoint_speed))   
                        control_el.setpoint_speed /= 1000;   // Convert to deg/s
                } else if (dataPtr[0] == 'V' && dataPtr[1] == 'D') {
                    // Elevation decrease speed in mdeg/s ("VDf" -> none)
                    rotator.control_mode = speed;
                    if (parse_field(dataPtr + 2, control_el.setpoint_speed))    
                        control_el.setpoint_speed /= -1000;   // Convert to deg/s and negate
                } else if (dataPtr[0] == 'V' && dataPtr[1] == 'L') {
                    // Azimuth increase speed in mdeg/s ("VLf" -> none)
                    rotator.control_mode = speed;
                    if (parse_field(dataPtr + 2, control_az.setpoint_speed))    
                        control_az.setpoint_speed /= 1000;   // Convert to deg/s
                } else if (dataPtr[0] == 'V' && dataPtr[1] == 'R') {
                    // Azimuth decrease speed in mdeg/s ("VRf" -> none)
                    rotator.control_mode = speed;
                    if (parse_field(dataPtr + 2, control_az.setpoint_speed))   
                        control_az.setpoint_speed /= -1000;   // Convert to deg/s and negate
                } else if (dataPtr[0] == 'S' && dataPtr[1] == 'A' && dataPtr[2] == ' ' && dataPtr[3] == 'S' && dataPtr[4] == 'E') {
                    // Stop Moving ("SA SE" -> "AZf ELf")
                    rotator.control_mode = position;
                    str1 = String("AZ");
                    str2 = String(control_az.input, 1);
                    str3 = String(" EL");
                    str4 = String(control_el.input, 1);
                    str5 = String("\n");
                    Serial.print(str1 + str2 + str3 + str4 + str5);
                    control_az.setpoint = control_az.input;
                    control_el.setpoint = control_el.input;
                } else if (dataPtr[0] == 'R' && dataPtr[1] == 'E' && dataPtr[2] == 'S' && dataPtr[3] == 'E' && dataPtr[4] == 'T') {
                    // Reset the rotator, go to home position ("RESET" -> "AZf ELf")
                    str1 = String("AZ");
                    str2 = String(control_az.input, 1);
                    str3 = String(" EL");
                    str4 = String(control_el.input, 1);
                    str5 = String("\n");
                    Serial.print(str1 + str2 + str3 + str4 + str5);
                    rotator.home_complete_flag = false;
                } else if (dataPtr[0] == 'P' && dataPtr[1] == 'A' && dataPtr[2] == 'R' && dataPtr[3] == 'K' ) {
                    // Park the rotator ("PARK" -> "AZf ELf")
                    rotator.control_mode = position;
                    str1 = String("AZ");
                    str2 = String(control_az.input, 1);
                    str3 = String(" EL");
                    str4 = String(control_el.input, 1);
                    str5 = String("\n");
                    Serial.print(str1 + str2 + str3 + str4 + str5);
                    control_az.setpoint = rotator.park_az;
                    control_el.setpoint = rotator.park_el;
                } else if (dataPtr[0] == 'A' && dataPtr[1] == 'T') {
                    // Autotune the PI loops ("AT" -> none)
                    rotator.autotune_pending_flag = true;
                } else if (dataPtr[0] == 'V' && dataPtr[1] == 'E') {
                    // Get the version if rotator controller ("VE" -> "VEs")
                    str1 = String("VE");
                    str2 = String("SatNOGS-v2.2");
                    str3 = String("\n");
                    Serial.print(str1 + str2 + str3);
                } else if (dataPtr[0] == 'I' && dataPtr[1] == 'P' && dataPtr[2] == '0') {
                    // Get the inside temperature ("IP0" -> "IP0,n")
                    str1 = String("IP0,");
                    str2 = String(rotator.inside_temperature, DEC);
                    str3 = String("\n");
                    Serial.print(str1 + str2 + str3);
                } else if (dataPtr[0] == 'I' && dataPtr[1] == 'P' && dataPtr[2] == '1') {
                    // Get the status of end-stop, azimuth ("IP1" -> "IP1,n")
                    str1 = String("IP1,");
                    str2 = String(rotator.switch_az, DEC);
                    str3 = String("\n");
                    Serial.print(str1 + str2 + str3);
                } else if (dataPtr[0] == 'I' && dataPtr[1] == 'P' && dataPtr[2] == '2') {
                    // Get the status of end-stop, elevation ("IP2" -> "IP2,n")
                    str1 = String("IP2,");
                    str2 = String(rotator.switch_el, DEC);
                    str3 = String("\n");
                    Serial.print(str1 + str2 + str3);
                } else if (dataPtr[0] == 'I' && dataPtr[1] == 'P' && dataPtr[2] == '3') {
                    // Get the current position of azimuth in deg ("IP3" -> "IP3,f")
                    str1 = String("IP3,");
                    str2 = String(control_az.input, 2);
                    str3 = String("\n");
                    Serial.print(str1 + str2 + str3);
                } else if (dataPtr[0] == 'I' && dataPtr[1] == 'P' && dataPtr[2] == '4') {
                    // Get the current position of elevation in deg ("IP4" -> "IP4,f")
                    str1 = String("IP4,");
                    str2 = String(control_el.input, 2);
                    str3 = String("\n");
                    Serial.print(str1 + str2 + str3);
                } else if (dataPtr[0] == 'I' && dataPtr[1] == 'P' && dataPtr[2] == '5') {
                    // Get the load of azimuth, in range of 0-1023 ("IP5" -> "IP5,n")
                    str1 = String("IP5,");
                    str2 = String(control_az.load, DEC);
                    str3 = String("\n");
                    Serial.print(str1 + str2 + str3);
                } else if (dataPtr[0] == 'I' && dataPtr[1] == 'P' && dataPtr[2] == '6') {
                    // Get the load of elevation, in range of 0-1023 ("IP6" -> "IP6,n")
                    str1 = String("IP6,");
                    str2 = String(control_el.load, DEC);
                    str3 = String("\n");
                    Serial.print(str1 + str2 + str3);
                } else if (dataPtr[0] == 'I' && dataPtr[1] == 'P' && dataPtr[2] == '7') {
                    // Get the speed of azimuth in deg/s ("IP7" -> "IP7,f")
                    str1 = String("IP7,");
                    str2 = String(control_az.speed, 2);
                    str3 = String("\n");
                    Serial.print(str1 + str2 + str3);
                } else if (dataPtr[0] == 'I' && dataPtr[1] == 'P' && dataPtr[2] == '8') {
                    // Get the speed of elevation in deg/s ("IP8" -> "IP8,f")
                    str1 = String("IP8,");
                    str2 = String(control_el.speed, 2);
                    str3 = String("\n");
                    Serial.print(str1 + str2 + str3);
                } else if (dataPtr[0] == 'G' && dataPtr[1] == 'S') {
                    // Get the status of rotator ("GS" -> "GSn")
                    str1 = String("GS");
                    str2 = String(rotator.rotator_status, DEC);
                    str3 = String("\n");
                    Serial.print(str1 + str2 + str3);
                } else if (dataPtr[0] == 'G' && dataPtr[1] == 'E') {
                    // Get the error of rotator ("GE" -> "GEn")
                    str1 = String("GE");
                    str2 = String(rotator.rotator_error, DEC);
                    str3 = String("\n");
                    Serial.print(str1 + str2 + str3);
                } else if(dataPtr[0] == 'C' && dataPtr[1] == 'R') {
                    // Get Configuration of rotator
                    if (dataPtr[3] == '1') {
                        // Get Kp Azimuth gain ("CR,1" -> "1,f")
                        str1 = String("1,");
                        str2 = String(control_az.p, 2);
                        str3 = String("\n");
                        Serial.print(str1 + str2 + str3);
                    } else if (dataPtr[3] == '2') {
                        // Get Ki Azimuth gain ("CR,2" -> "2,f")
                        str1 = String("2,");
                         str2 = String(control_az.i, 2);
                         str3 = String("\n");
                         Serial.print(str1 + str2 + str3);
                    } else if (dataPtr[3] == '3') {
                        // Get Kd Azimuth gain ("CR,3" -> "3,f")
                        str1 = String("3,");
                        str2 = String(control_az.d, 2);
                        str3 = String("\n");
                        Serial.print(str1 + str2 + str3);
                    } else if (dataPtr[3] == '4') {
                        // Get Kp Elevation gain ("CR,4 -> "4,f")
                        str1 = String("4,");
                         str2 = String(control_el.p, 2);
                         str3 = String("\n");
                         Serial.print(str1 + str2 + str3);
                    } else if (dataPtr[3] == '5') {
                        // Get Ki Elevation gain ("CR,5" -> "5,f")
                        str1 = String("5,");
                        str2 = String(control_el.i, 2);
                        str3 = String("\n");
                        Serial.print(str1 + str2 + str3);
                    } else if (dataPtr[3] == '6') {
                        // Get Kd Elevation gain ("CR,6" -> "6,f")
                        str1 = String("6,");
                        str2 = String(control_el.d, 2);
                        str3 = String("\n");
                        Serial.print(str1 + str2 + str3);
                    } else if (dataPtr[3] == '7') {
                        // Get Azimuth park position ("CR,7" -> "7,f")
                        str1 = String("7,");
                        str2 = String(rotator.park_az, 2);
                        str3 = String("\n");
                        Serial.print(str1 + str2 + str3);
                    } else if (dataPtr[3] == '8') {
                        // Get Elevation park position ("CR,8" -> "8,f")
                        str1 = String("8,");
                        str2 = String(rotator.park_el, 2);
                        str3 = String("\n");
                        Serial.print(str1 + str2 + str3);
                    } else if (dataPtr[3] == '9') {
                        // Get control mode ("CR,9" -> "9,f")
                        str1 = String("9,");
                        str2 = String(rotator.control_mode);
                        str3 = String("\n");
                        Serial.print(str1 + str2 + str3);
                    }
                } else if (dataPtr[0] == 'C' && dataPtr[1] == 'W') {
                    // Set Config
                    if (dataPtr[2] == '1') {
                        // Set Kp Azimuth gain ("CW1,f" -> none)
                        parse_field(dataPtr + 4, control_az.p);
                    } else if (dataPtr[2] == '2') {
                        // Set Ki Azimuth gain ("CW2,f" -> none)
                        parse_field(dataPtr + 4, control_az.i);
                    } else if (dataPtr[2] == '3') {
                        // Set Kd Azimuth gain ("CW3,f" -> none)
                        parse_field(dataPtr + 4, control_az.d);
                    } else if (dataPtr[2] == '4') {
                        // Set Kp Elevation gain ("CW4,f" -> none)
                        parse_field(dataPtr + 4, control_el.p);
                    } else if (dataPtr[2] == '5') {
                        // Set Ki Elevation gain ("CW5,f" -> none)
                        parse_field(dataPtr + 4, control_el.i);
                    } else if (dataPtr[2] == '6') {
                        // Set Kd Elevation gain ("CW6,f" -> none)
                        parse_field(dataPtr + 4, control_el.d);
                    }  else if (dataPtr[2] == '7') {
                        // Set the Azimuth park position ("CW7,f" -> none)
                        parse_field(dataPtr + 4, rotator.park_az);
                    } else if (dataPtr[2] == '8') {
                        // Set the Elevation park position ("CW8,f" -> none)
                        parse_field(dataPtr + 4, rotator.park_el);
                    }
                } else if (dataPtr[0] == 'R' && dataPtr[1] == 'S' && dataPtr[2] == 'T') {
                    // Custom command to test the watchdog timer routine  ("RST" -> none)
                    while(1)
                        ;
                } else if (dataPtr[0] == 'R' && dataPtr[1] == 'B') {
                    // Custom command to reboot the uC ("RB" -> none)
                    wdt_enable(WDTO_2S);
                    while(1);
                }
                // Reset the buffer and clean the serial buffer
                BufferCnt = 0;
            } else {
                // Fill the buffer with incoming data
                buffer[BufferCnt] = incomingByte;
                BufferCnt++;
                // Prevent buffer overrun
                if (BufferCnt > BUFFER_SIZE)
                {
                    BufferCnt = 0;
                    Serial.flush();
                }
            }
        }
    }

private:

    static int const BUFFER_SIZE = 40;   ///< Set the size of serial buffer
    static int const DATA_SIZE = 10;     ///< Set the size of data buffer

    char buffer[BUFFER_SIZE+1]; 
    uint16_t BufferCnt = 0;

    static char * safe_strncpy(char *dest, char const *src, size_t const size) {
        strncpy(dest, src, size-1);
        dest[size-1] = '\0';
        return dest;
    }

    static bool parse_field(char const *src, double& val) {
        char data[DATA_SIZE+1];
        safe_strncpy(data, src, DATA_SIZE);
        if (isNumber(data)) {
            val = atof(data);
            return true;    // Success
        }
        return false;   // Failed
    }

    static bool isNumber(char *input) {
        for (uint16_t i = 0; input[i] != '\0'; i++) {
            if (isalpha(input[i]))
                return false;
        }
        return true;
    }
};

#endif /* LIBRARIES_EASYCOMM_H_ */
