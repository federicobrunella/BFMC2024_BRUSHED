/*
 * Configuration.h
 *
 *  Created on: Oct 27, 2023
 *      Author: M.Cristina Giannini
 */

#ifndef INC_CONFIGURATION_H_
#define INC_CONFIGURATION_H_

#include "math.h"

//PRESS USER BUTTON
#define SHORT_PRESS_THRESHOLD  100 //Sono multipli di 10ms
#define LONG_PRESS_THRESHOLD  200 //Sono multipli di 10ms

//PWM
#define MIN_PWM -2.6
#define MAX_PWM 2.6
#define NEUTRAL_PWM 0

//TRACTION PID
#define TRACTION_SAMPLING_TIME 0.01 //[s]
#define MAX_U_TRACTION MAX_PWM // Valore di duty [% del periodo]
#define MIN_U_TRACTION MIN_PWM // Valore di duty [% del periodo]

//TRACTION PID FRONTE
#define KP_TRACTION 0.0011//0.000015 //0.0001
#define KI_TRACTION 0.0045 //0.0030 //0.003

//STEERING PID
#define STEERING_SAMPLING_TIME 0.01 //[s]
#define MAX_U_STEERING 23 // [°]
#define MIN_U_STEERING -23 //[°]
#define KP_STEERING 20//1.8//1.25
#define KI_STEERING 250//500//166//250

//ENCODER SAMPLING TIME
#define ENCODER_SAMPLING_TIME 0.01 //[s]

//VEHICLE PARAMETERS for RPM<->m/s
#define WHEEL_RADIUS 0.03//m
#define MOTOR_REVOLUTION_FOR_ONE_WHEEL_REVOLUTION 5

#define MAX_CURVATURE_RADIUS_FOR_STRAIGHT 10


#endif /* INC_CONFIGURATION_H_ */
