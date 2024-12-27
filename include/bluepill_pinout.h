/*
 * \file bluepill_pinout.h
 * \package Hibachi
 * This file is part of Hibachi Firmware
 *
 * \author     Agustín Capovilla <acapovilla@sinc.unl.edu.ar>
 * \copyright  Copyright (c) 2021-2022, ___ORGANIZATION___.
 *
 * Developed for ___ORGANIZATION___
 * See the LICENSE file at the top-level directory of this distribution
 * for details of code ownership.
 *
 * Please send comments, questions, or patches to ___ISSUE_MAIL___
 */

#ifndef INCLUDE_BLUEPILL_PINOUT_H__
#define INCLUDE_BLUEPILL_PINOUT_H__

#define FRONT_LEFT_MOTOR_DIR \
    PA1  // direction control for left hand motor AIN1 pin on motor controller
#define FRONT_LEFT_MOTOR_SLEEP \
    PA2  // direction control for left hand motor AIN2 pin on motor controller
#define FRONT_LEFT_MOTOR_PWM PA7  // PWM pin for left hand motor A

#define FRONT_RIGHT_MOTOR_DIR \
    PB7  // direction control for right hand motor AIN1 pin on motor controller
#define FRONT_RIGHT_MOTOR_SLEEP \
    PB6  // direction control for right hand motor AIN2 pin on motor controller
#define FRONT_RIGHT_MOTOR_PWM PB9  // PWM pin for right hand motor A

#define REAR_LEFT_MOTOR_DIR \
    PA4  // direction control for left hand motor BIN1 pin on motor controller
#define REAR_LEFT_MOTOR_SLEEP \
    PA3  // direction control for left hand motor BIN2 pin on motor controller
#define REAR_LEFT_MOTOR_PWM PB0  // PWM pin for left hand motor B

#define REAR_RIGHT_MOTOR_DIR \
    PB4  // direction control for right hand motor BIN1 pin on motor controller
#define REAR_RIGHT_MOTOR_SLEEP \
    PB5  // direction control for right hand motor BIN2 pin on motor controller
#define REAR_RIGHT_MOTOR_PWM PB8  // PWM pin for right hand motor B

// Pins for the encoder inputs
#define FRONT_LEFT_ENCODER_A  PA8
#define FRONT_LEFT_ENCODER_B  PA9

#define FRONT_RIGHT_ENCODER_A PB14
#define FRONT_RIGHT_ENCODER_B PB15

#define REAR_LEFT_ENCODER_A   PB10
#define REAR_LEFT_ENCODER_B   PB11

#define REAR_RIGHT_ENCODER_A  PB12
#define REAR_RIGHT_ENCODER_B  PB13

// Encoders ticks per rev
#define TICKS_PER_REV 4480 / 4

#endif  // !INCLUDE_BLUEPILL_PINOUT_H__
