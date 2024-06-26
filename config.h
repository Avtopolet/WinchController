/*
 * config.h
 *
 *  Automatically created by OSEC Parameter Configurator
 *  Author: stancecoke
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#define NUMBER_OF_PAS_MAGS 10
#define limit 25
#define timeout 3125
#define wheel_circumference 2040L
#define limit_without_pas 6
#define ADC_THROTTLE_MIN_VALUE 80
#define ADC_THROTTLE_MAX_VALUE 185
#define BATTERY_VOLTAGE_MIN_VALUE 100
#define BATTERY_CURRENT_MAX_VALUE 150L
#define PHASE_CURRENT_MAX_VALUE 500L
#define REGEN_CURRENT_MAX_VALUE 0L
#define MOTOR_ROTOR_DELTA_PHASE_ANGLE_RIGHT 237
#define current_cal_a 76
#define LEVEL_1 30
#define LEVEL_2 45
#define LEVEL_3 60
#define LEVEL_4 80
#define LEVEL_5 100
#define MORSE_TIME_1 50
#define MORSE_TIME_2 50
#define MORSE_TIME_3 50
#define RAMP_END 1500
#define P_FACTOR 0.5
#define I_FACTOR 0.1
#define GEAR_RATIO 106L
#define PAS_THRESHOLD 1.9
#define RAMP_START 64000 //64000
#define limit_with_throttle_override 40
#define CORRECTION_AT_ANGLE 127
#define DIAGNOSTICS
#define ANGLE_4_0 1
#define ANGLE_6_60 43
#define ANGLE_2_120 86
#define ANGLE_3_180 128
#define ANGLE_1_240 171
#define ANGLE_5_300 213
#define TQS_CALIB 0.0
#define ACA 4736
#define EEPROM_NOINIT // eeprom will not be cleared
#define EEPROM_INIT_MAGIC_BYTE 25 // makes sure (chance of fail 1/255) eeprom is invalidated after flashing new config
#define ADC_BATTERY_VOLTAGE_K 69
#define ACA_EXPERIMENTAL 140
#define BATTERY_VOLTAGE_MAX_VALUE 208

#endif /* CONFIG_H_ */
