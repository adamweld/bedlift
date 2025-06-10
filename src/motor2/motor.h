// #include "Arduino.h"
#pragma once
#include "driver/twai.h"
#include "../cybergear/cybergear.h"
#include "../cybergear/cybergear_utils.h"
#include <M5_4Relay.h>
#include <Wire.h>

// Pins used to connect to CAN bus transceiver:
#define RX_PIN 1
#define TX_PIN 2

#define SDA_PIN 13
#define SCL_PIN 15

#define MASTER_CAN_ID 0x00

// Intervall:
#define TRANSMIT_RATE_MS 100
#define POLLING_RATE_MS 100

#define NUM_MOTORS 4
#define MOTOR_MODE MODE_SPEED
// #define MOTOR_MODE MODE_POSITION
#define MOTOR_LIMIT_SPEED 10.0f
// #define MOTOR_LIMIT_CURRENT 27.0 //max
#define MOTOR_LIMIT_CURRENT 7



class Motor
{
    public:
    // Cybergear motors
    cybergear_motor_t m1;
    cybergear_motor_t m2;
    cybergear_motor_t m3;
    cybergear_motor_t m4;

    cybergear_motor_t _motors[NUM_MOTORS] = {
        XiaomiCyberGearDriver(0x01, MASTER_CAN_ID),
        XiaomiCyberGearDriver(0x02, MASTER_CAN_ID),
        XiaomiCyberGearDriver(0x03, MASTER_CAN_ID),
        XiaomiCyberGearDriver(0x04, MASTER_CAN_ID),
    };

    // Relay
    M5_4Relay relay;

    // Variables
    uint8_t _rx_can_id;

public:
    Motor()
    {
    }
    void init();
    void update();
    void check_alerts();
    void handle_rx_message(twai_message_t& message);
    void set_speed(float speed, int m_id);
    void enable(int m_id);
    void disable(int m_id);
    void unlock(int m_id);
    void lock(int m_id);
    void reset();
};