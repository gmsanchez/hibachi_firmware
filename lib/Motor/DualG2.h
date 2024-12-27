/*
 * \file DualG2.h
 * \package DualG2
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

#ifndef MOTOR_DUALG2_H__
#define MOTOR_DUALG2_H__

#include "Motor.h"

/**
 * @brief Abstracción de un motor para control desde un puente-H Dual G2
 * con 3 señales:
 *  - 1 señal analógica para controlar la velocidad
 *  - 1 señal digital para controlar el sentido de giro
 *  - 1 señal para activación de la salida
 *
 * Para establecer la velocidad la clase consta de un mapeo PWM, que se
 * inicializa en el constructor, para controlar el rango de operación del
 * motor.
 * @see PWM_map
 *
 * @author Capovilla Agustín
 * @version 0.0.1
 * @since 2022-04
 */
class DualG2 : public Motor {
   public:
    /**
     * Inicializa los GPIO de control del puente-H como salidas y establece el
     * mapeo de control del PWM
     *
     * @param[in] enable_pin GPIO analógico para control de velocidad
     * @param[in] dir_pin GPIO digital para control del sentido de giro
     * @param[in] sleep_pin GPIO digital para activación de las salidas
     * @param[in] pwm_map Mapeo PWM para el control de la velocidad
     * @param[in] fliped Flag para indicar si los comandos de dirección deben
     * invertirse
     */
    DualG2(const int enable_pin, const int dir_pin, const int sleep_pin,
           const PWM_map pwm_map, bool fliped = false, bool negated_logic = false)
        : Motor(pwm_map),
          _enable_pin(enable_pin),
          _dir_pin(dir_pin),
          _sleep_pin(sleep_pin),
          _fliped(fliped),
          _negated_logic(negated_logic){};

    /**
     * Inicializa los GPIO de control del puente-H como salidas y establece
     * el controlador como activo (pin de SLEEP en HIGH)
     */
    void init(void) {
        pinMode(_enable_pin, OUTPUT);
        analogWrite(_enable_pin, 0);
        
        pinMode(_dir_pin, OUTPUT);
        pinMode(_sleep_pin, OUTPUT);

        digitalWrite(_sleep_pin, _negated_logic ? HIGH : LOW);
        _sleeping = true;

        // The drivers require a maximum of 1ms to elapse when brought out of
        // sleep mode.
        delay(5);
    }

    /**
     * Rotar en sentido hacia adelante
     */
    void setForward(void) {
        digitalWrite(_dir_pin, _fliped ? HIGH : LOW);
        digitalWrite(_sleep_pin, _negated_logic ? LOW : HIGH);
        if (_sleeping) {
            delay(5);
            _sleeping = false;
        }
    }

    /**
     * Rotar en sentido hacia atrás
     */
    void setBackward(void) {
        digitalWrite(_dir_pin, _fliped ? LOW : HIGH);
        digitalWrite(_sleep_pin, _negated_logic ? LOW : HIGH);
        if (_sleeping) {
            delay(5);
            _sleeping = false;
        }
    }

    /**
     * Liberar
     */
    void setFree(void) {
        digitalWrite(_sleep_pin, _negated_logic ? HIGH : LOW);
        _sleeping = true;
    }

    /**
     * Establecer la velocidad a partir de un mapeo PWM
     * el cual es convertido al mapeo del motor
     *
     * @param[in] pwm_value Valor de PWM a esablecer
     */
    void setPWM(const PWM pwm_value) {
        pwm.setPWM(pwm_value);
        analogWrite(_enable_pin, pwm.getValue());
    }

   private:
    int _enable_pin, _dir_pin, _sleep_pin;  // GPIO de control del puente-H
    bool _fliped;                           // Flip direction
    bool _negated_logic;                     // Inverse enable pin logic
    bool _sleeping;
};

#endif  // !MOTOR_DUALG2_H__