/*
 * \file Encoder.h
 * \package Encoder
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

#ifndef ENCODER_ENCODER_H__
#define ENCODER_ENCODER_H__

#ifndef M_PI
  #define M_PI 3.14159265358979323846
#endif

/**
 * @brief Abstracción de un encoder incremental de cuadratura.
 *
 * Con el método `updateCount` el decodificador de cuadratura (resolución
 * 4X) calcula la posición y actualiza el ángulo a partir del desplazamiento
 * angular establecido en el constructor de la clase.
 * La posición del encoder (en radianes): @f$0 < \alpha < 2\pi@f$
 *
 * @author Capovilla Agustín
 * @version 0.0.1
 * @since 2021-12
 */
class Encoder {
   public:
    /**
     * Inicializa el encoder en la posición 0
     *
     * @param[in] rad_per_tick Desplazamiento angular en radianes por cada
     * pulso del encoder.
     */
    Encoder(const float rad_per_tick) : _rad_per_tick(rad_per_tick) {
        _angle = 0;
        _cw_direction = 0;

        state_history = 0;
        decoder_state = 0;
    };

    /**
     * Obtener la posición del encoder
     *
     * @return Posición angular en radianes
     *
     * @note @f$0 < \alpha < 2\pi@f$
     */
    float getPosition(void) const { return _angle; }

    /**
     * Obtener la posición acumulada del encoder
     *
     * @return Posición angular en radianes
     */
    float getPositionUnwrap(void) const { return _angle_unwrap; }

    /**
     * Obtener el sentido de rotación del encoder
     *
     * @return Si es `true` el sentido es horario,
     * si es `false` el sentido es antihorario
     */
    bool getDirection(void) const { return _cw_direction; }

    /**
     * Reestablecer la posición del encoder
     *
     * @param[in] angle Angulo en radianes entre 0 y 2 PI
     * (valor por defecto: 0 rad)
     *
     * @warning @f$0 < \alpha < 2\pi@f$
     */
    void resetPosition(const float angle = 0.0f) {
        if (angle < 0.0f || angle > (2 * M_PI)) return;
        _angle = angle;
    }

    /**
     * Calcular la posición del encoder en base al nuevo estado.
     * Esta función debe ser llamada cada vez que el estado de la
     * señal de cualquiera de los dos canales cambie, dado que el
     * decodificador de cuadratura opera en resolución 4X. Se
     * recomienda utilizar `external hardware interrupts` en modo
     * `CHANGE`.
     *
     * @param[in] channel_A Estado de la señal del canal A (LOW o HIGH)
     * @param[in] channel_B Estado de la señal del canal B (LOW o HIGH)
     */
    void updateCount(const bool channel_A, const bool channel_B) {
        // Quadrature decoder lookup table (4X resolution)
        constexpr static signed char _lut[16] = {0,  -1, 1, 0, 1, 0, 0,  -1,
                                                 -1, 0,  0, 1, 0, 1, -1, 0};

        unsigned char s = state_history << 2;  // Save old state
        // Add new state
        s |= channel_A << 1;
        s |= channel_B;
        state_history = (s & 0b11);  // Save for next step

        decoder_state += _lut[s];  // Quadrature decoder based en LUT

        if (decoder_state < -3) {  // 4 counts backward = 1 pulse backward
            countDown();
            decoder_state = 0;           // Reset decoder state
        } else if (decoder_state > 3) {  // 4 counts forward = 1 pulse forward
            countUp();
            decoder_state = 0;  // Reset decoder state
        }
    };

   private:
    signed char decoder_state;    // Accum for counts
    unsigned char state_history;  // 4X encoder count state

    float _rad_per_tick;  // Angle per pulse (4 ticks) in radians
    float _angle;         // Encoder position
    double _angle_unwrap;

    bool _cw_direction;

    void countUp(void) {
        _cw_direction = 1;
        _angle += _rad_per_tick;  // Add displacement
        _angle_unwrap += _rad_per_tick;
        if (_angle > (2 * M_PI)) {  // and clamp value
            _angle -= (2 * M_PI);
        }
    }

    void countDown(void) {
        _cw_direction = 0;
        _angle -= _rad_per_tick;  // Add displacement
        _angle_unwrap -= _rad_per_tick;
        if (_angle < 0.0f) {  // and clamp value
            _angle += (2 * M_PI);
        }
    }
};

#undef M_PI

#endif  // !ENCODER_ENCODER_H__