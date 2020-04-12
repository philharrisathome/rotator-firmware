/*!
* @file watchdog.h
*
* It is a driver for watchdog timer.
*
* Licensed under the GPLv3
*
*/

#ifndef WATCHDOG_H_
#define WATCHDOG_H_

#include <avr/wdt.h>
#include "globals.h"
#include "easycomm.h"
#include "rotator_pins.h"

/**************************************************************************/
/*!
    @brief    Class that functions for interacting with a watchdog timer
*/
/**************************************************************************/
class wdt_timer{
public:

    /**************************************************************************/
    /*!
        @brief    Initialize watchdog timer to 2sec time out and to set up
                  interrupt routine
    */
    /**************************************************************************/
    void watchdog_init() {
        cli();
        wdt_reset();
        WDTCSR |= _BV(WDCE) | _BV(WDE);
        WDTCSR = _BV(WDIE) | _BV(WDE) | _BV(WDP3) | _BV(WDP2) | _BV(WDP1);
        sei();
    }

    /**************************************************************************/
    /*!
        @brief    Reset the watchdog timer
    */
    /**************************************************************************/
    void watchdog_reset() {
        wdt_reset();
    }
};

/**************************************************************************/
/*!
    @brief    Watchdog timer interrupt routine that implements a minimal
              easycomm protocol to get the errors and reset the uC
*/
/**************************************************************************/
ISR(WDT_vect) {
    // Disable motors
    // TODO:
    // Set error
    rotator.rotator_error = wdt_error;
    rotator.rotator_status = error;
    // Enable interrupts for serial communication
    sei();

    Serial.println("**** PANIC! ****");     // TODO:

    int const BUFFER_SIZE(10);

    while (1) {
        // Reset the watchdog timer because the interrupts are enabled
        wdt_reset();
        // Implement a minimal easycomm protocol to get the errors and reset uC
        char buffer[BUFFER_SIZE];           // TODO: Should be outside loop
        char incomingByte;
        static uint16_t BufferCnt = 0;      // TODO: Why is this static? Should be outside loop
        String str1, str2, str3, str4, str5, str6;
        while (Serial.available() > 0) {
            incomingByte = Serial.read();
            if (incomingByte == '\n' || incomingByte == '\r') {
                buffer[BufferCnt] = 0;
                if (buffer[0] == 'G' && buffer[1] == 'S') {
                    // Get the status of rotator ("GS" -> "GSn")
                    str1 = String("GS");
                    str2 = String(rotator.rotator_status, DEC);
                    str3 = String("\n");
                    Serial.print(str1 + str2 + str3);
                } else if (buffer[0] == 'G' && buffer[1] == 'E') {
                    // Get the error of rotator ("GE" -> "GEn")
                    str1 = String("GE");
                    str2 = String(rotator.rotator_error, DEC);
                    str3 = String("\n");
                    Serial.print(str1 + str2 + str3);
                } else if (buffer[0] == 'R' && buffer[1] == 'B') {
                    // Custom command to reboot the uC ("RB" -> none)
                    while(1);
                }
                BufferCnt = 0;
                Serial.flush();
            } else {
                buffer[BufferCnt] = incomingByte;
                BufferCnt++;
                // Prevent buffer overrun
                if (BufferCnt >= BUFFER_SIZE)
                {
                    BufferCnt = 0;
                    Serial.flush();
                }            
            }
        }
        // Reset the watchdog timer
        wdt_reset();
    }
}

#endif /* WATCHDOG_H_ */
