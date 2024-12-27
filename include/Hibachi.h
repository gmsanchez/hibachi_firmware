/*
 * \file Hibachi.h
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

#ifndef INCLUDE_HIBACHI_H__
#define INCLUDE_HIBACHI_H__

#include <Encoder.h>
#include <PWM.h>
#include <DualG2.h>
#include <SpeedEstimator.h>
#include <PID_SpeedController.h>
#include <SerialHDLC.h>

//-------------------------------------------------------------------
// Wiring pinout and specific board configs
//-------------------------------------------------------------------

#include "blackpill_pinout.h"

//-------------------------------------------------------------------
// Mechanic constants
//-------------------------------------------------------------------

const double m_pi = 3.14159265358979323846;

// Encoder
const int ticks_per_rev = TICKS_PER_REV;
const double rad_per_tick = (2 * m_pi) / double(ticks_per_rev);

// Gain values for PID motor control initialization
#define DEFAULT_PID_kP 25.0f
#define DEFAULT_PID_kI 1.0f
#define DEFAULT_PID_kD 5.0f

// Delta T [mS] for speed estimation and PID adjustment
#define PID_UPDATE_mS 20
const int pid_freq_update_hz = 1000 / PID_UPDATE_mS;

// Control stale timeout for safey
#define CMD_TIMEOUT_mS 500
const int timeout_ovf = CMD_TIMEOUT_mS / PID_UPDATE_mS;
static int timeout_counter = 0;
static bool timeout_flag = false;

#define PWM_OUT_RESOLUTION     12  // From 0 to 4095
#define PWM_MOTOR_MAP_MAXVALUE ((1 << PWM_OUT_RESOLUTION) - 1)
// Valores para inicializar el mapeo PWM de los motores
#define PWM_MOTOR_MAP                 \
    {.min = 0,                        \
     .max = PWM_MOTOR_MAP_MAXVALUE,   \
     .dbMin = 0,                      \
     .dbMax = PWM_MOTOR_MAP_MAXVALUE, \
     .center = 0}
// Valores para inicializar el mapeo PWM de los PID
#define PWM_PID_MAP                   \
    {.min = 0,                        \
     .max = PWM_MOTOR_MAP_MAXVALUE,   \
     .dbMin = 0,                      \
     .dbMax = PWM_MOTOR_MAP_MAXVALUE, \
     .center = 0}

//-------------------------------------------------------------------
// Encoder and motors globals
//-------------------------------------------------------------------

// Front left
extern DualG2 front_left_motor;
extern Encoder front_left_encoder;
extern SpeedEstimator front_left_speed_estimator;
extern PID_SpeedController front_left_speed_control;

// Front right
extern DualG2 front_right_motor;
extern Encoder front_right_encoder;
extern SpeedEstimator front_right_speed_estimator;
extern PID_SpeedController front_right_speed_control;

// Rear left
extern DualG2 rear_left_motor;
extern Encoder rear_left_encoder;
extern SpeedEstimator rear_left_speed_estimator;
extern PID_SpeedController rear_left_speed_control;

// Rear right
extern DualG2 rear_right_motor;
extern Encoder rear_right_encoder;
extern SpeedEstimator rear_right_speed_estimator;
extern PID_SpeedController rear_right_speed_control;

/** Setear los pines como entradas y setear las interrupciones
 * con las correspondientes funciones los encoders */
void setupEncoders(void);

/** Invocar las rutinas de inicializacion de los motores */
void setupMotors(void);

/** Inicializar los PID de los controladores de velocidad
 * según los valores de ganancia definidos por defecto */
void setupSpeedControllers(void);

/** Rutina de ajuste de velocidad */
void adjustSpeeds(void);

//-------------------------------------------------------------------
// Serial communication globals
//-------------------------------------------------------------------
extern SerialHDLC hdlc;

void onHandleFrame(const uint8_t* frameBuffer, uint16_t frameLength);

#endif  // !INCLUDE_HIBACHI_H__
