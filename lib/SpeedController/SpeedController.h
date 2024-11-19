/*
 * \file SpeedController.h
 * \package SpeedController
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

#ifndef SPEEDCONTROLLER_SPEEDCONTROLLER_H__
#define SPEEDCONTROLLER_SPEEDCONTROLLER_H__

#include "SpeedEstimator.h"
#include "PWM.h"
#include "Motor.h"

/**
 * @brief Abstracción de un controlador de velocidad de lazo cerrado basado
 * en los valores arrojados por el estimador de velocidad y los comandos
 * enviados al motor. A través del método `setSpeed` se establece el setpoint
 * del controlador.
 *
 * El método `adjustSpeed` ejecuta los cálculos correspondientes al lazo de
 * control, por lo que debe ser llamado de forma periódica a intervalos
 * constantes de duración igual al valor especificado en el `SpeedEstimator`
 * @see SpeedEstimator
 * @see Motor
 *
 * @author Capovilla Agustín
 * @version 0.0.1
 * @since 2021-12
 */
class SpeedController {
   public:
    /**
     * @brief Inicializa la clase con las referencias al `SpeedEstimator` y al
     * `Motor`. Establece el mapeo PWM que será utilizado como interfaz de
     * control del motor. Incializa el setpoint en 0.
     *
     * @param[in] speed_estimator Estimado de velocidad a utilizar
     * @param[in] motor Motor a controlar
     * @param[in] pwm_map Mapeo PWM
     */
    SpeedController(SpeedEstimator* speed_estimator, Motor* motor,
                    const PWM_map pwm_map)
        : _motor(motor), _pwm(pwm_map), _speed(speed_estimator) {
        _setPoint = 0;

        reset();
    };

    /**
     * @brief Establece la velocidad del motor en 0 y reestablece
     * las variables utilizadas para el cálculo del lazo de control
     */
    virtual void reset(void) { setSpeed(0.0); }

    /**
     * @brief Establecer un nuevo setpoint para el controlador.
     *
     * @param[in] speed Velocidad en [rad/s]
     */
    virtual void setSpeed(float speed) { _setPoint = speed; };

    /**
     * @brief Ajustar la velocidad del motor en base al controlador.
     *
     * @note Este método ejecuta los cálculos del lazo de control
     * por lo que debe ejecutarse de forma periódica en base al
     * intervalo establecido en el estimador de velocidad
     */
    virtual void adjustSpeed(void) = 0;

    /**
     * @brief Obtener la velocidad calculada por el estimador de
     * velocidad
     *
     * @return Velocidad en [rad/s]
     */
    float getSpeed(void) { return _speed->getSpeed(); }

   protected:
    // Referencia para anviar comandos al motor
    Motor* _motor;

    // Interfaz PWM para controlar el motor
    PWM _pwm;

    // Estimador de velocidad para obtener el feedback del sistema
    SpeedEstimator* _speed;

    // Setpoint del controlador
    float _setPoint;
};

#endif  // !SPEEDCONTROLLER_SPEEDCONTROLLER_H__