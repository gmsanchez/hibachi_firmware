/*
 * \file Hibachi.cpp
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

#include "Hibachi.h"

//-------------------------------------------------------------------
// Encoder and motors initialization
//-------------------------------------------------------------------
PWM_map defaultPWM_motor_map = PWM_MOTOR_MAP;
// Valores para inicializar el mapeo PWM de los PID
PWM_map defaultPWM_PID_map = PWM_PID_MAP;

DualG2 front_left_motor(FRONT_LEFT_MOTOR_PWM, FRONT_LEFT_MOTOR_DIR,
                        FRONT_LEFT_MOTOR_SLEEP, defaultPWM_motor_map, true);
Encoder front_left_encoder(rad_per_tick);
SpeedEstimator front_left_speed_estimator(PID_UPDATE_mS);
PID_SpeedController front_left_speed_control(&front_left_speed_estimator,
                                             &front_left_motor,
                                             defaultPWM_PID_map);

DualG2 front_right_motor(FRONT_RIGHT_MOTOR_PWM, FRONT_RIGHT_MOTOR_DIR,
                         FRONT_RIGHT_MOTOR_SLEEP, defaultPWM_motor_map, false);
Encoder front_right_encoder(rad_per_tick);
SpeedEstimator front_right_speed_estimator(PID_UPDATE_mS);
PID_SpeedController front_right_speed_control(&front_right_speed_estimator,
                                              &front_right_motor,
                                              defaultPWM_PID_map);

DualG2 rear_left_motor(REAR_LEFT_MOTOR_PWM, REAR_LEFT_MOTOR_DIR,
                       REAR_LEFT_MOTOR_SLEEP, defaultPWM_motor_map, true);
Encoder rear_left_encoder(rad_per_tick);
SpeedEstimator rear_left_speed_estimator(PID_UPDATE_mS);
PID_SpeedController rear_left_speed_control(&rear_left_speed_estimator,
                                            &rear_left_motor,
                                            defaultPWM_PID_map);

DualG2 rear_right_motor(REAR_RIGHT_MOTOR_PWM, REAR_RIGHT_MOTOR_DIR,
                        REAR_RIGHT_MOTOR_SLEEP, defaultPWM_motor_map, false);
Encoder rear_right_encoder(rad_per_tick);
SpeedEstimator rear_right_speed_estimator(PID_UPDATE_mS);
PID_SpeedController rear_right_speed_control(&rear_right_speed_estimator,
                                             &rear_right_motor,
                                             defaultPWM_PID_map);

/** Interrupt handler para los encoders
 *      - Actualizan el contador de la clase encoder (tipo incremental)
 *      - Actualizan el estimador de velocidad en base a la nueva posición
 */
void read_front_left_encoder(void) {
    front_left_encoder.updateCount(digitalRead(FRONT_LEFT_ENCODER_A),
                                   digitalRead(FRONT_LEFT_ENCODER_B));
    front_left_speed_estimator.updatePosition(
        front_left_encoder.getPosition(), front_left_encoder.getDirection());
}

void read_front_right_encoder(void) {
    front_right_encoder.updateCount(digitalRead(FRONT_RIGHT_ENCODER_B),
                                    digitalRead(FRONT_RIGHT_ENCODER_A));
    front_right_speed_estimator.updatePosition(
        front_right_encoder.getPosition(), front_right_encoder.getDirection());
}

void read_rear_left_encoder(void) {
    rear_left_encoder.updateCount(digitalRead(REAR_LEFT_ENCODER_A),
                                  digitalRead(REAR_LEFT_ENCODER_B));
    rear_left_speed_estimator.updatePosition(rear_left_encoder.getPosition(),
                                             rear_left_encoder.getDirection());
}

void read_rear_right_encoder(void) {
    rear_right_encoder.updateCount(digitalRead(REAR_RIGHT_ENCODER_B),
                                   digitalRead(REAR_RIGHT_ENCODER_A));
    rear_right_speed_estimator.updatePosition(
        rear_right_encoder.getPosition(), rear_right_encoder.getDirection());
}

void setupEncoders(void) {
    pinMode(FRONT_LEFT_ENCODER_A, INPUT);
    pinMode(FRONT_LEFT_ENCODER_B, INPUT);
    attachInterrupt(digitalPinToInterrupt(FRONT_LEFT_ENCODER_A),
                    read_front_left_encoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(FRONT_LEFT_ENCODER_B),
                    read_front_left_encoder, CHANGE);

    pinMode(FRONT_RIGHT_ENCODER_A, INPUT);
    pinMode(FRONT_RIGHT_ENCODER_B, INPUT);
    attachInterrupt(digitalPinToInterrupt(FRONT_RIGHT_ENCODER_A),
                    read_front_right_encoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(FRONT_RIGHT_ENCODER_B),
                    read_front_right_encoder, CHANGE);

    pinMode(REAR_LEFT_ENCODER_A, INPUT);
    pinMode(REAR_LEFT_ENCODER_B, INPUT);
    attachInterrupt(digitalPinToInterrupt(REAR_LEFT_ENCODER_A),
                    read_rear_left_encoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(REAR_LEFT_ENCODER_B),
                    read_rear_left_encoder, CHANGE);

    pinMode(REAR_RIGHT_ENCODER_A, INPUT);
    pinMode(REAR_RIGHT_ENCODER_B, INPUT);
    attachInterrupt(digitalPinToInterrupt(REAR_RIGHT_ENCODER_A),
                    read_rear_right_encoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(REAR_RIGHT_ENCODER_B),
                    read_rear_right_encoder, CHANGE);
}

void setupMotors(void) {
    front_left_motor.init();
    front_right_motor.init();
    rear_left_motor.init();
    rear_right_motor.init();
}

void setupSpeedControllers(void) {
    front_left_speed_control.setGains(DEFAULT_PID_kP, DEFAULT_PID_kI,
                                      DEFAULT_PID_kD);

    front_right_speed_control.setGains(DEFAULT_PID_kP, DEFAULT_PID_kI,
                                       DEFAULT_PID_kD);

    rear_left_speed_control.setGains(DEFAULT_PID_kP, DEFAULT_PID_kI,
                                     DEFAULT_PID_kD);

    rear_right_speed_control.setGains(DEFAULT_PID_kP, DEFAULT_PID_kI,
                                      DEFAULT_PID_kD);
}

void adjustSpeeds(void) {
    front_left_speed_control.adjustSpeed();
    front_right_speed_control.adjustSpeed();
    rear_left_speed_control.adjustSpeed();
    rear_right_speed_control.adjustSpeed();
}

//-------------------------------------------------------------------
// Serial communication rutines
//-------------------------------------------------------------------
SerialHDLC hdlc(onHandleFrame, 32);

uint8_t getWheelEncoderResponse[12] = {0x04, 0x70, 0x08, 0x00, 0x00, 0x00,
                                       0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

void setWheelControl(const uint8_t* frameBuffer, uint16_t frameLength) {
    if (frameLength == 8) {
        // Rebuild data for each wheel
        int16_t front_left_cmd =
            ((int16_t)frameBuffer[1] << 8) | frameBuffer[0];
        int16_t front_right_cmd =
            ((int16_t)frameBuffer[3] << 8) | frameBuffer[2];
        int16_t rear_left_cmd = ((int16_t)frameBuffer[5] << 8) | frameBuffer[4];
        int16_t rear_right_cmd =
            ((int16_t)frameBuffer[7] << 8) | frameBuffer[6];

        // From INT to FLOAT (2 decimals)
        float front_left_speed_cmd = (float)front_left_cmd / 100.0f;
        float front_right_speed_cmd = (float)front_right_cmd / 100.0f;
        float rear_left_speed_cmd = (float)rear_left_cmd / 100.0f;
        float rear_right_speed_cmd = (float)(rear_right_cmd) / 100.0f;

        if (front_left_speed_cmd == 0.0f && front_right_speed_cmd == 0.0f &&
            rear_left_speed_cmd == 0.0f && rear_right_speed_cmd == 0.0f) {
            timeout_flag = false;
        }

        // Assign new speed
        front_left_speed_control.setSpeed(front_left_speed_cmd);
        front_right_speed_control.setSpeed(front_right_speed_cmd);
        rear_left_speed_control.setSpeed(rear_left_speed_cmd);
        rear_right_speed_control.setSpeed(rear_right_speed_cmd);
    }
}

void setPWMOutput(const uint8_t* frameBuffer, uint16_t frameLength) {
    if (frameLength == 4) {
        PWM_map pwm_map_test = PWM_map{
            .min = 0, .max = 254, .dbMin = 0, .dbMax = 254, .center = 0};
        PWM pwm_(pwm_map_test);

        for (int idx = 0; idx < 4; ++idx) {
            uint8_t _cmd = (int8_t)frameBuffer[idx] & 0xFF;
            if (_cmd == 0) {
                switch (idx) {
                    case 0:
                        front_left_motor.setFree();
                        break;
                    case 1:
                        front_right_motor.setFree();
                        break;
                    case 2:
                        rear_left_motor.setFree();
                        break;
                    case 3:
                        rear_right_motor.setFree();
                        break;

                    default:
                        break;
                }
            } else {
                pwm_.setPWM(_cmd);
                switch (idx) {
                    case 0:
                        front_left_motor.setForward();
                        front_left_motor.setPWM(pwm_);
                        break;
                    case 1:
                        front_right_motor.setForward();
                        front_right_motor.setPWM(pwm_);
                        break;
                    case 2:
                        rear_left_motor.setForward();
                        rear_left_motor.setPWM(pwm_);
                        break;
                    case 3:
                        rear_right_motor.setForward();
                        rear_right_motor.setPWM(pwm_);
                        break;

                    default:
                        break;
                }
            }
        }
    }
}

void getWheelEncoder(const uint8_t* frameBuffer, uint16_t frameLength) {
    //  The order is front_left_wheel, front_right_wheel, rear_left_wheel,
    //  rear_right_wheel

    /** ---------- Position ---------- */
    getWheelEncoderResponse[0] = 0x04;
    // From INT to FLOAT (2 decimals)
    int16_t front_left_wheel_data =
        (int16_t)(front_left_encoder.getPosition() * 100.0);
    int16_t front_right_wheel_data =
        (int16_t)(front_right_encoder.getPosition() * 100.0);
    int16_t rear_left_wheel_data =
        (int16_t)(rear_left_encoder.getPosition() * 100.0);
    int16_t rear_right_wheel_data =
        (int16_t)(rear_right_encoder.getPosition() * 100.0);

    getWheelEncoderResponse[4] = (uint8_t)((front_left_wheel_data) & 0xFF);
    getWheelEncoderResponse[5] = (uint8_t)((front_left_wheel_data >> 8) & 0xFF);
    getWheelEncoderResponse[6] = (uint8_t)((front_right_wheel_data) & 0xFF);
    getWheelEncoderResponse[7] =
        (uint8_t)((front_right_wheel_data >> 8) & 0xFF);
    getWheelEncoderResponse[8] = (uint8_t)((rear_left_wheel_data) & 0xFF);
    getWheelEncoderResponse[9] = (uint8_t)((rear_left_wheel_data >> 8) & 0xFF);
    getWheelEncoderResponse[10] = (uint8_t)((rear_right_wheel_data) & 0xFF);
    getWheelEncoderResponse[11] =
        (uint8_t)((rear_right_wheel_data >> 8) & 0xFF);

    hdlc.write(getWheelEncoderResponse, 12);

    /** ---------- Speed ---------- */
    getWheelEncoderResponse[0] = 0x06;
    // From INT to FLOAT (2 decimals)
    front_left_wheel_data =
        (int16_t)(front_left_speed_control.getSpeed() * 100.0);
    front_right_wheel_data =
        (int16_t)(front_right_speed_control.getSpeed() * 100.0);
    rear_left_wheel_data =
        (int16_t)(rear_left_speed_control.getSpeed() * 100.0);
    rear_right_wheel_data =
        (int16_t)(rear_right_speed_control.getSpeed() * 100.0);

    getWheelEncoderResponse[4] = (uint8_t)((front_left_wheel_data) & 0xFF);
    getWheelEncoderResponse[5] = (uint8_t)((front_left_wheel_data >> 8) & 0xFF);
    getWheelEncoderResponse[6] = (uint8_t)((front_right_wheel_data) & 0xFF);
    getWheelEncoderResponse[7] =
        (uint8_t)((front_right_wheel_data >> 8) & 0xFF);
    getWheelEncoderResponse[8] = (uint8_t)((rear_left_wheel_data) & 0xFF);
    getWheelEncoderResponse[9] = (uint8_t)((rear_left_wheel_data >> 8) & 0xFF);
    getWheelEncoderResponse[10] = (uint8_t)((rear_right_wheel_data) & 0xFF);
    getWheelEncoderResponse[11] =
        (uint8_t)((rear_right_wheel_data >> 8) & 0xFF);

    hdlc.write(getWheelEncoderResponse, 12);
}

void resetEncoders(void) {
    front_left_encoder.resetPosition();
    front_right_encoder.resetPosition();
    rear_left_encoder.resetPosition();
    rear_right_encoder.resetPosition();
}

void resetSpeedEstimators(void) {
    front_left_speed_estimator.reset();
    front_right_speed_estimator.reset();
    rear_left_speed_estimator.reset();
    rear_right_speed_estimator.reset();
}

void resetSpeedControllers(void) {
    front_left_speed_control.reset();
    front_right_speed_control.reset();
    rear_left_speed_control.reset();
    rear_right_speed_control.reset();
}

void setGainsSpeedControllers(const uint8_t* frameBuffer,
                              uint16_t frameLength) {
    // First byte = wheel number ( 0: All, 1: FL, 2: FR, 3: RL, 4: RR)
    // 3 gains * 2 bytes each = 6 bytes
    if (frameLength == 7) {
        // Wheel code
        uint8_t _wh = frameBuffer[0];

        // Rebuild data for each gain
        int16_t _kP_data = ((int16_t)frameBuffer[2] << 8) | frameBuffer[1];
        int16_t _kI_data = ((int16_t)frameBuffer[4] << 8) | frameBuffer[3];
        int16_t _kD_data = ((int16_t)frameBuffer[6] << 8) | frameBuffer[5];

        // From INT to FLOAT (3 decimals)
        float _kP = (float)_kP_data / 1000.0f;
        float _kI = (float)_kI_data / 1000.0f;
        float _kD = (float)_kD_data / 1000.0f;

        // Assign new gains
        switch (_wh) {
            case 0:
                front_left_speed_control.setGains(_kP, _kI, _kD);
                front_right_speed_control.setGains(_kP, _kI, _kD);
                rear_left_speed_control.setGains(_kP, _kI, _kD);
                rear_right_speed_control.setGains(_kP, _kI, _kD);
                break;
            case 1:
                front_left_speed_control.setGains(_kP, _kI, _kD);
                break;

            case 2:
                front_right_speed_control.setGains(_kP, _kI, _kD);
                break;

            case 3:
                rear_left_speed_control.setGains(_kP, _kI, _kD);
                break;

            case 4:
                rear_right_speed_control.setGains(_kP, _kI, _kD);
                break;
            default:
                break;
        }
    }
}

void onHandleFrame(const uint8_t* frameBuffer, uint16_t frameLength) {
    if (frameLength >= 2) {
        uint16_t cmd =
            (((uint16_t)frameBuffer[0]) | (((uint16_t)frameBuffer[1]) << 8));
        uint16_t len =
            (((uint16_t)frameBuffer[2]) | (((uint16_t)frameBuffer[3]) << 8));
        switch (cmd) {
            case 0x0001:
                // setWheelControl(&frameBuffer[4], len);
                break;
            case 0x0002:
                // getWheelEncoder(&frameBuffer[4], len);
                break;
            case 0x0003:
                // getMPUData(&frameBuffer[4], len);
                break;
            case 0x0004:
                getWheelEncoder(&frameBuffer[4], len);
                break;
            case 0x0005:
                timeout_flag = true;
                timeout_counter = 0;
                setWheelControl(&frameBuffer[4], len);
                break;
            case 0x0105:
                setPWMOutput(&frameBuffer[4], len);
                break;
            case 0x0006:  // Odometry reset
                resetEncoders();
                resetSpeedEstimators();
                resetSpeedControllers();

                timeout_counter = 0;
                timeout_flag = false;
                break;
            case 0x0010:  // Set PID gains value
                setGainsSpeedControllers(&frameBuffer[4], len);
            case 0x00AA:
                hdlc.write(frameBuffer, frameLength);
                break;
        }
    }
}