// #include "Arduino.h"
#pragma once
#include "driver/twai.h"
#include "../xiaomi_cybergear/xiaomi_cybergear_driver.h"
#include "../4relay/Unit_4RELAY.h"

// Pins used to connect to CAN bus transceiver:
#define RX_PIN 1
#define TX_PIN 2

#define MASTER_CAN_ID 0x00

// Intervall:
#define TRANSMIT_RATE_MS 100
#define POLLING_RATE_MS 100

#define NUM_MOTORS 1
#define MOTOR_MODE MODE_SPEED
// #define MOTOR_MODE MODE_POSITION
#define MOTOR_LIMIT_SPEED 10.0f
#define MOTOR_LIMIT_CURRENT 5.0


class Motor
{
    public:
    // Cybergear motors
    XiaomiCyberGearDriver _motors[NUM_MOTORS] = {
        XiaomiCyberGearDriver(0x01, MASTER_CAN_ID),
    };

    // Relay
    UNIT_4RELAY relay;

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
};