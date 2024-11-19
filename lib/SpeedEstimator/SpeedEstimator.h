/*
 * \file SpeedEstimator.h
 * \package SpeedEstimator
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

#ifndef SPEEDESTIMATOR_SPEEDESTIMATOR_H__
#define SPEEDESTIMATOR_SPEEDESTIMATOR_H__

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/**
 * @brief Estimador de velocidad a partir de un encoder el cual
 * reporta su posición en radianes y el cálculo se realiza a intervalos
 * constantes definidos en el constructor.
 *
 * @author Capovilla Agustín
 * @version 0.0.1
 * @since 2021-12
 */
class SpeedEstimator {
   public:
    /**
     * Inicializa la clase y establece el valor del intervalo de tiempo en el
     * que se va a realizar la estimación.
     *
     * El valor de timeout en que la posición se debe mantener constante para
     * determinar que el motor se encuentra detenido, se calcula en cantidad de
     * intervalos.
     *
     * @param[in] win_span Intervalo de tiempo en milisegundos
     * @param[in] timeout_acc Cantidad de intervalos para establecer timeout.
     */
    SpeedEstimator(const unsigned int win_span,
                   const unsigned int timeout_acc = 10)
        : _win_span(win_span), _timeout_acc(timeout_acc) {
        reset();
    };

    void reset(void) {
        _new_position = 0.0f;
        _cw_direction = true;

        speed = 0.0f;
        win_acc = 1;
        old_position = 0.0f;
    }

    /**
     * Obtener la velocidad estimada
     *
     * @return Velocidad en radianes por segundo (signed)
     */
    float getSpeed(void) { return _cw_direction ? speed : -speed; }

    /**
     * Estima y devuelve la velocidad en valor absoluto
     *
     * @return Velocidad en radianes por segundo
     */
    float calcSpeed(void) {
        if (old_position == _new_position) {  // Si la posición no cambió
            ++win_acc;                        // Se contabiliza el intervalo

            // Timeout: cuando el motor se encuentra detenido, la posición
            // no cambia, por lo que el contador seguiría acumulando. Para no
            // tener un overflow, se mantiene en un valor máximo
            if (win_acc > _timeout_acc) {  // Si se cumplió el tiempo
                speed = 0.0f;              // Establecer la velocidad en cero
                win_acc =
                    _timeout_acc;  // Mantener el contador en el valor máximo
            }
        } else {  // Si cambió
                  // Se calcula el delta en [rad]
            float dPosition = _new_position - old_position;

            if (!_cw_direction) {
                dPosition *= -1;
            }

            // Actualizar a la nueva posición para la siguiente vez
            old_position = _new_position;

            if (dPosition < 0.0f) {     // Cuando la diferencia es negativa
                dPosition += 2 * M_PI;  // El delta real es el águlo más chico
            }

            // El delta T son la cantidad de intervalos que transcurrieron
            unsigned int dTime = win_acc * _win_span;  // [mS]
            win_acc = 1;  // Resetear el contador de intervalos

            // Convertir el valor de intervalo en [mS] a 1/T[S]
            float dTime_sec = 1000.0 / (double)dTime;

            // Actualizar la velocidad estimada
            speed = dPosition * dTime_sec;  //[rad/S]
        }

        return speed;
    }

    /**
     * Actualizar la posición del encoder
     *
     * @param[in] new_position Ángulo en radianes
     */
    void updatePosition(const float new_position, const bool cw_direction) {
        _new_position = new_position;  // Establecer la nueva posición
        _cw_direction = cw_direction;
    }

   private:
    // Intervalo de tiempo entre el cálculo de velocidad
    unsigned int _win_span;

    // Límite superior para el contador de intevalos
    unsigned int _timeout_acc;

    // Variable para calcular la diferencia de posición
    float _new_position;

    // Velocidad estimada
    float speed;

    // Contador de intervalos mientras la posición no cambió
    unsigned int win_acc;

    // Posición anterior para detectar variación
    float old_position;

    bool _cw_direction;
};

#undef M_PI

#endif  // !SPEEDESTIMATOR_SPEEDESTIMATOR_H__