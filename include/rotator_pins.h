/*!
* @file rotator_pins.h
*
* It is a header file for pin mapping.
*
* Licensed under the GPLv3
*
*/

#ifndef ROTATOR_PINS_H_
#define ROTATOR_PINS_H_

int const AZIMUTH_HOME(6);
int const AZIMUTH_PHASE_A(2);
int const AZIMUTH_PHASE_B(4);
int const ELEVATION_HOME(7);
int const ELEVATION_PHASE_A(3);
int const ELEVATION_PHASE_B(5);

int const AZIMUTH_DRIVE_IN1(A2);
int const AZIMUTH_DRIVE_IN2(A3);
int const AZIMUTH_DRIVE_EN(9);
int const ELEVATION_DRIVE_IN1(A0);
int const ELEVATION_DRIVE_IN2(A1);
int const ELEVATION_DRIVE_EN(10);

#endif /* ROTATOR_PINS_H_ */
