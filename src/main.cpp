#include <Arduino.h>
#include "Hibachi.h"

HardwareTimer Timer2(TIM2);  // Referencia para configuración del Timer 2

// Prescaler y variable de estado para blink del led PC13 como testigo
// de ejecución del loop de control de velocidad
volatile bool led_state = false;
volatile uint16_t soft_prescaler = 0;

/**
 * Rutina de interrupción para el Timer 2. Invoca el método adjustSpeeds() que
 * ejecuta el ajuste de cada lazo de control de velocidad
 */
void timer2_interrupt(void) {
    adjustSpeeds();

    // Prescaler para blink a menor frecuencia (visible)
    if (soft_prescaler > 2) {
        soft_prescaler = 0;

        led_state = !led_state;
        digitalWrite(LED_BUILTIN, led_state);
    } else {
        ++soft_prescaler;
    }

    // Velocity command timeout
    if (timeout_counter++ >= timeout_ovf) {
        if (timeout_flag) {
            front_left_speed_control.reset();
            front_right_speed_control.reset();
            rear_left_speed_control.reset();
            rear_right_speed_control.reset();
            timeout_flag = false;
            timeout_counter = 0;
        }
        timeout_counter = timeout_ovf;
    }
}

/**
 * Setear el Timer 2 para generar interrupciones a 50 [Hz] (período de 20 ms)
 * La rutina de manejo de interrupción será `timer2_interrupt()`
 */
void setupTimer2(void) {
    Timer2.pause();
    Timer2.setOverflow(50, HERTZ_FORMAT);
    Timer2.attachInterrupt(timer2_interrupt);
    Timer2.resume();
}

void setup() {
    Serial.begin(115200);
    while (!Serial)  // No iniciar hasta obtener conexión por USB
        ;

    setupEncoders();  // Crear interrupts de los encoders
    setupMotors();    // Setear salidas para puentes-H

    setupSpeedControllers();  // Inicializar controladores de velocidad

    // Establecer la resolución de salida según el mapeo PWM establecido
    // para el motor (12bits -> [0, 4095])
    analogWriteResolution(PWM_OUT_RESOLUTION);

    analogWriteFrequency(500);

    pinMode(LED_BUILTIN, OUTPUT);  // Usar el LED como testigo de ejecución

    // Setear el TIMER2 para generar interrupciones para ejecutar el ajuste
    // de los lazo de control de velocidad de los motores
    setupTimer2();

    Serial.println("<Arduino is ready>");
}

void loop() {
    hdlc.read();
}