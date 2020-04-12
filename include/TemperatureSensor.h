/*!
* @file TemperatureSensor.h
*
* It is a driver for a Temperature sensor.
*
* Licensed under the GPLv3
*
*/

#ifndef TemperatureSensor_H_
#define TemperatureSensor_H_

#include <Wire.h>
#include <RtcDS3231.h>

void printDateTime(const RtcDateTime& dt)
{
    char datestring[20];

    snprintf_P(datestring, 
            sizeof(datestring),
            PSTR("%02u/%02u/%04u %02u:%02u:%02u"),
            dt.Month(),
            dt.Day(),
            dt.Year(),
            dt.Hour(),
            dt.Minute(),
            dt.Second() );
    Serial.print(datestring);
}

/**************************************************************************/
/*!
    @brief    Class that functions for interacting with a Temperature sensor
*/
/**************************************************************************/
class TemperatureSensor {
public:

    TemperatureSensor() : _rtc(Wire) {
    }

    /**************************************************************************/
    /*!
        @brief    Initialize the sensor
    */
    /**************************************************************************/
    void init() {
        _rtc.Begin();

        RtcDateTime compiled = RtcDateTime(__DATE__, __TIME__);
        printDateTime(compiled);
        Serial.println();

        if (!_rtc.IsDateTimeValid()) 
        {
            Serial.println("RTC lost confidence in the DateTime!");
            _rtc.SetDateTime(compiled);
        }

        if (!_rtc.GetIsRunning())
        {
            Serial.println("RTC was not actively running, starting now");
            _rtc.SetIsRunning(true);
        }

        RtcDateTime now = _rtc.GetDateTime();
        printDateTime(now);
        Serial.println();
        if (!now.IsValid())
        {
            // Common Causes:
            //    1) the battery on the device is low or even missing and the power line was disconnected
            Serial.println("RTC lost confidence in the DateTime!");
        }    

        if (now < compiled) 
        {
            Serial.println("RTC is older than compile time!  (Updating DateTime)");
            _rtc.SetDateTime(compiled);
        }
    }

    /**************************************************************************/
    /*!
        @brief    Reads the int8_t in temperature measurement register
        @return   The temperature int8_t
    */
    /**************************************************************************/
    int8_t get_temp() {
        RtcTemperature temp = _rtc.GetTemperature();
        return int(temp.AsFloatDegC());
    }

    /**************************************************************************/
    /*!
        @brief    Reads the int8_t in status register
        @return   The status register int8_t
    */
    /**************************************************************************/
    int8_t get_status() {
        return 0;
    }

    /**************************************************************************/
    /*!
        @brief    Wake up request to the sensor on the specified address
        @return   The status register int8_t
    */
    /**************************************************************************/
    int8_t wake_up() {
        // nothing to do
        return get_status();
    }

    /**************************************************************************/
    /*!
        @brief    Sleep device request to the sensor on the specified address
        @return   The status register int8_t
    */
    /**************************************************************************/
    int8_t sleep() {
        // nothing to do
        return get_status();
    }

private:
    RtcDS3231<TwoWire> _rtc;
};

#endif /* TemperatureSensor_H_ */
