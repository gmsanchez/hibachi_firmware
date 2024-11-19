/*
 * \file L298N.h
 * \package L298N
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

#ifndef MOTOR_L298N_H__
#define MOTOR_L298N_H__

#include "Motor.h"

/**
 * @brief Abstracción de un motor para control desde un puente-H L298N
 * con 3 señales:
 *  - 1 señal analógica para controlar la velocidad
 *  - 2 señales digitales para controlar el sentido de giro
 *
 * Además del giro hacia adelante y atrás, se definen dos estados adicionales:
 * frenado (`inA` HIGH y `inB` HIGH) y libre (`inA` LOW y `inB` LOW).
 *
 * Para establecer la velocidad la clase consta de un mapeo PWM, que se
 * inicializa en el constructor, para controlar el rango de  operación del
 * motor.
 * @see PWM_map
 *
 * @author Capovilla Agustín
 * @version 0.0.1
 * @since 2022-04
 */
class L298N : public Motor {
   public:
    /**
     * Establece los GPIO de control del L298N y el mapeo del PWM para el
     * control de la velocidad
     *
     * @param[in] enable_pin L298N [EN] pin (analógico)
     * @param[in] inA_pin L298N [INA] pin (digital)
     * @param[in] inB_pin L298N [INB] pin (digital)
     * @param[in] pwm_map Mapeo PWM para el control de la velocidad
     */
    L298N(const int enable_pin, const int inA_pin, const int inB_pin,
          const PWM_map pwm_map)
        : Motor(pwm_map),
          _enable_pin(enable_pin),
          _inA_pin(inA_pin),
          _inB_pin(inB_pin){};

    /**
     * Establece las GPIO de control como salidas y en apagadas (LOW)
     */
    void init(void) {
        pinMode(_enable_pin, OUTPUT);
        pinMode(_inA_pin, OUTPUT);
        pinMode(_inB_pin, OUTPUT);

        digitalWrite(_enable_pin, LOW);
        digitalWrite(_inA_pin, LOW);
        digitalWrite(_inB_pin, LOW);
    }

    /**
     * Establece el sentido de giro hacia adelante
     */
    void setForward(void) {
        digitalWrite(_inA_pin, HIGH);
        digitalWrite(_inB_pin, LOW);
    }

    /**
     * Establece el sentido de giro hacia atrás
     */
    void setBackward(void) {
        digitalWrite(_inA_pin, LOW);
        digitalWrite(_inB_pin, HIGH);
    }

    /**
     * Desconectar o liberar las salidas del driver
     */
    void setFree(void) {
        digitalWrite(_inA_pin, LOW);
        digitalWrite(_inB_pin, LOW);
    }

    /**
     * Realizar un frenado
     */
    void setStop(void) {
        digitalWrite(_inA_pin, HIGH);
        digitalWrite(_inB_pin, HIGH);
        digitalWrite(_enable_pin, HIGH);
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
    int _enable_pin, _inA_pin, _inB_pin;  // GPIO de control del puente-H
};

#endif  // !MOTOR_L298N_H__