/*!
* @file MotorEncoder.h
*
* Licensed under the GPLv3
*
*/

#ifndef MotorEncoder_H_
#define MotorEncoder_H_

#include <Encoder.h>

/**************************************************************************/
/*!
    @brief    Class that functions for motor encoder.
*/
/**************************************************************************/
class MotorEncoder : private Encoder{
public:

    /**************************************************************************/
    /*!
        @brief    Construct the encoder
    */
    /**************************************************************************/
    MotorEncoder(uint8_t phaseApin, uint8_t phaseBpin) : Encoder(phaseApin, phaseBpin) {
    }

    /**************************************************************************/
    /*!
        @brief    Initialize the encoder
    */
    /**************************************************************************/
    void Begin() {
    }

    /**************************************************************************/
    /*!
        @brief    Calculate an unwrap the position
        @param    new_pos
                  Calculate the current position of the sensor
        @return   The state of the encoder
    */
    /**************************************************************************/
    uint8_t get_pos(double *new_pos) {

        float const ENCODER_SCALE(25345.04);

        // Convert raw value to angle in deg
        float raw_pos = (360.0f * read() / ENCODER_SCALE);
        // Calculate the real angle
        float real_pos = - (raw_pos / _enc_ratio) - _angle_offset;
        *new_pos = (double)real_pos;
        return 0;
    }

    /**************************************************************************/
    /*!
        @brief    Set zero by setting offset angle
        @return   The state of the encoder
    */
    /**************************************************************************/
    uint8_t set_zero() {
        double current_pos;
        uint8_t status_val = get_pos(&current_pos);
        _angle_offset = current_pos;
        return status_val;
    }

    /**************************************************************************/
    /*!
        @brief    Reset zero position set the offset to zero
    */
    /**************************************************************************/
    void init_zero() {
        _angle_offset = 0.0;
    }

    /**************************************************************************/
    /*!
        @brief    Set the gear ratio between encoder and measure axis
        @param    enc_ratio
                  An uitn8_t, that represents the gear ratio
    */
    /**************************************************************************/
    void set_gear_ratio(uint8_t enc_ratio) {
        _enc_ratio = enc_ratio;
    }

private:
    double _angle_offset = 0;
    uint8_t _enc_ratio = 0;
};

#endif /* MotorEncoder_H_ */

