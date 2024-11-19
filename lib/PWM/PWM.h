/*
 * \file PWM.h
 * \package PWM
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

#ifndef PWM_PWM_H__
#define PWM_PWM_H__

/**
 * @brief Tipo de dato que encapsula todos los parámetros necesarios para
 * definir un mapeo PWM. Estos son:
 *  - `min`: Valor mínimo del rango
 *  - `max`: Valor máximo del rango
 *  - `dbMin`: Valor mínimo de operación (deadband)
 *  - `dbMax`: Valor máximo de operación (deadband)
 *  - `center`: Valor medio dentro del rango de operación
 *
 * @warning Todos los datos son enteros de tipo `int`
 */
typedef struct {
    int min, max, dbMin, dbMax, center;
} PWM_map;

/**
 * @brief Implementación del mapeo entre distintos rangos de PWM. Esto es útil
 * cuando se necesita convertir un rango de valores desde una función de control
 * a un valor capaz de ser reproducido por el hardware con cierta resolución.
 *
 * @author Capovilla Agustín
 * @version 0.0.1
 * @since 2021-12
 *
 * @warning Todos los datos y operaciones se realizan entre enteros de tipo
 * `int`
 */
class PWM {
   public:
    /**
     * Inicializa la clase a partir de la configuración enviada. Incializa el
     * valor del PWM en el valor de `PWM_map::center`.
     *
     * @param[in] config Configuración del mapeo PWM
     */
    PWM(const PWM_map config) {
        _map = config;
        _pwm = _map.center;
    };

    /**
     * Setear el valor del PWM. Si el mismo se encuentra fuera del rango de
     * operación, se setea el valor mínimo o máximo según corresponda.
     *
     * @param[in] value Nuevo valor de PWM
     */
    void setPWM(const int value) {
        // Clamp to deadband range
        _pwm = clampPWM(value);
    }

    /**
     * Setear el valor del PWM en base a un mapeo dado (se presupone que los
     * rangos de operación se encuentran superpuestos y existe un mapeo entre
     * ellos). El nuevo valor se encontrará dentro del rango de operación
     * definido.
     *
     * @param[in] value Nuevo valor de PWM
     *
     * @overload
     */
    void setPWM(const PWM value) {
        _pwm = clampPWM(mapPWM(value._pwm, value._map.min, value._map.max,
                               _map.min, _map.max));
    }

    /**
     * Obtener el valor de PWM
     *
     * @return Valor entero del PWM
     */
    int getValue(void) const { return _pwm; }

    /**
     * Obtener la configuración del mapeo
     *
     * @return Mapeo PWM
     */
    PWM_map getMap(void) const { return _map; }

   private:
    /**
     * Mapear un valor entre dos rangos conocidos
     * this is the map() function that is currently in the ESP8266 platform core
     * library
     *
     * @return Valor mapeado entre los rango
     *
     * @see
     * [WMath.cpp](https://github.com/esp8266/Arduino/blob/master/cores/esp8266/WMath.cpp)
     */
    static long mapPWM(const long value, const long fromMin, const long fromMax,
                       const long toMin, const long toMax) {
        const long dividend = toMax - toMin;
        const long divisor = fromMax - fromMin;
        const long delta = value - fromMin;
        return (delta * dividend + (divisor / 2)) / divisor + toMin;
    }

    // Mantener el valor de PWM en el rango de operación
    int clampPWM(const int value) const {
        if (value < _map.dbMin)
            return _map.dbMin;
        else if (value > _map.dbMax)
            return _map.dbMax;
        else
            return value;
    }

    int _pwm;      // Valor del PWM
    PWM_map _map;  // Parámetros de configuración del mapeo
};

#endif  // !PWM_PWM_H__