/*
 * \file Motor.h
 * \package Motor
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

#ifndef MOTOR_MOTOR_H__
#define MOTOR_MOTOR_H__

#include "PWM.h"

#include <Arduino.h>

/**
 * @brief Abstracción de un motor para control desde un puente-H, con
 * métodos para establecer la dirección de giro y la velocidad.
 *
 * Para establecer la velocidad la clase consta de un mapeo PWM, que se
 * inicializa en el constructor, para controlar el rango de  operación del
 * motor.
 * @see PWM_map
 *
 * @author Capovilla Agustín
 * @version 0.1.0
 * @since 2021-12
 */
class Motor {
   public:
    /**
     * Establece el mapeo de control del PWM
     *
     * @param[in] pwm_map Mapeo PWM para el control de la velocidad
     */
    Motor(const PWM_map pwm_map) : pwm(pwm_map){};

    /**
     * Inicializar los parámetros y GPIO necesarias para el control
     * del puente-H
     */
    virtual void init(void) = 0;

    /**
     * Establece el sentido de giro hacia adelante
     */
    virtual void setForward(void) = 0;

    /**
     * Establece el sentido de giro hacia atrás
     */
    virtual void setBackward(void) = 0;

    /**
     * Desconectar o liberar las salidas del driver
     */
    virtual void setFree(void) = 0;

    /**
     * Establecer la velocidad a partir de un mapeo PWM
     * el cual es convertido al mapeo del motor
     *
     * @param[in] pwm_value Valor de PWM a esablecer
     */
    virtual void setPWM(const PWM pwm_value) = 0;

   protected:
    PWM pwm;  // Mapeo PWM
};

#endif  // !MOTOR_MOTOR_H__