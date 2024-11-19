/*
 * \file PID_SpeedController.h
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

#ifndef SPEEDCONTROLLER_PIDSPEEDCONTROLLER_H__
#define SPEEDCONTROLLER_PIDSPEEDCONTROLLER_H__

#include "SpeedController.h"

#define INTEG_MIN (-15.0f)
#define INTEG_MAX (15.0f)
/**
 * @brief Controlador de velocidad PID basado en los valores arrojados por el
 * estimador de velocidad y el comando de un motor. A través del método
 * `setSpeed` se establece el setpoint del controlador. El mismo puede ser
 * positivo o negativo, lo que permite establecer el sentido de giro del motor.
 *
 * El método `adjustSpeed` ejecuta los cálculos correspondientes al lazo de
 * control, por lo que debe ser llamado de forma periódica a intervalos
 * constantes, de igual duración que el valor especificado en el
 * `SpeedEstimator`
 *
 * Los parámetros kP, kI y kD son configurables de forma dinámica a través
 * del método `setGains` y los mismos se inicializan en 0.
 *
 * @author Capovilla Agustín
 * @version 0.0.1
 * @since 2022-07
 */
class PID_SpeedController : public SpeedController {
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
    PID_SpeedController(SpeedEstimator* speed_estimator, Motor* motor,
                        const PWM_map pwm_map)
        : SpeedController(speed_estimator, motor, pwm_map) {
        _kP = 0;
        _kI = 0;
        _kD = 0;
    };

    /**
     * @brief Establece la velocidad del motor en 0 y reestablece
     * las variables utilizadas para el cálculo del lazo de control
     */
    void reset(void) override {
        setSpeed(0.0);

        _iTerm = 0;
        last_speed = 0;
    }

    /**
     * @brief Establecer los valores de ganancia para el controlador PID
     *
     * @param[in] kP Ganancia del término proporcional
     * @param[in] kI Ganancia del término integral
     * @param[in] kD Ganancia del término derivativo
     */
    void setGains(const float kP, const float kI, const float kD) {
        _kP = kP;
        _kI = kI;
        _kD = kD;
    };

    /**
     * @brief Establecer un nuevo setpoint para el controlador. Si el mismo
     * es mayor a cero, la dirección del motor se establece hacia adelante y si
     * es menor a cero, la dirección se establece hacia atrás.
     * Por otro lado, si el setpoint es igual a 0, el motor se establece en
     * modo libre.
     *
     * @param[in] speed Velocidad en [rad/s]
     *
     * @note no se recomienda establecer setpoints de distinto signo de forma
     * consecutiva dado que puede dañar el motor
     */
    void setSpeed(float speed) override {
        if (speed < 0) {
            _motor->setBackward();
            speed *= -1;
        } else if (speed > 0) {
            _motor->setForward();
        }
        if (speed == 0) {
            _motor->setFree();
            _iTerm = 0;
            last_speed = 0;
            _pwm.setPWM(0);
        }
        _setPoint = speed;
    };

    /**
     * @brief Ajustar la velocidad del motor en base al controlador PID
     *
     * @note Este método ejecuta los cálculos del lazo de control
     * por lo que debe ejecutarse de forma periódica en base al
     * intervalo establecido en el estimador de velocidad
     */
    void adjustSpeed(void) override {
        float speed = _speed->calcSpeed();  // Velocidad estimada en [rad/s]
        float error = _setPoint - speed;    // Calular el error

        if (_setPoint == 0.0) return;

        // Integral
        _iTerm += (_kI * (float)error);

        // Limit sum to 32-bit signed value so that it saturates, never
        // overflows.
        if (_iTerm > INTEG_MAX)
            _iTerm = INTEG_MAX;
        else if (_iTerm < INTEG_MIN)
            _iTerm = INTEG_MIN;

        // Derivative
        float dInput = speed - last_speed;
        last_speed = speed;  // Actualizar la nueva velocidad

        // Proportional + Integral - Derivative
        signed long adjustment = (_kP * (float)error) + _iTerm - (_kD * dInput);

        _pwm.setPWM(_pwm.getValue() +
                    adjustment);  // Calcular el nuevo valor de PWM

        _motor->setPWM(_pwm);  // Establecer el nuevo valor
    };

   private:
    // PID aux
    float _iTerm, last_speed;

    // Parámetros del controlador PID
    float _kP, _kI, _kD;
};

#endif  // !SPEEDCONTROLLER_PIDSPEEDCONTROLLER_H__