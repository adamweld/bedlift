// #include "Arduino.h"
#pragma once
#include "driver/twai.h"
#include "esp_log.h"
#include "cybergear.h"
#include "cybergear_utils.h"
#include <M5_4Relay.h>
#include <Wire.h>

// Pins used to connect to CAN bus transceiver:
#define RX_PIN 1
#define TX_PIN 2

#define SDA_PIN 13
#define SCL_PIN 15

#define MASTER_CAN_ID 0x00

#define NUM_MOTORS 4

#define M1_CAN_ID 0x01
#define M2_CAN_ID 0x04
#define M3_CAN_ID 0x03
#define M4_CAN_ID 0x02


#define MOTOR_MODE CYBERGEAR_MODE_SPEED
// #define MOTOR_MODE MODE_POSITION
#define MOTOR_LIMIT_SPEED 10.0f
// #define MOTOR_LIMIT_CURRENT 27.0 //max
#define MOTOR_LIMIT_CURRENT 7
#define MOTOR_LIMIT_TORQUE 10

#define TWAI_ALERTS ( TWAI_ALERT_RX_DATA | \
		TWAI_ALERT_TX_IDLE | TWAI_ALERT_TX_SUCCESS | \
		TWAI_ALERT_TX_FAILED | TWAI_ALERT_ERR_PASS | \
		TWAI_ALERT_BUS_ERROR )

#define POLLING_RATE_TICKS pdMS_TO_TICKS(100)

#define TAG "motor"

class Motor
{
    public:
    // Cybergear motors
    cybergear_motor_t m1;
    cybergear_motor_t m2;
    cybergear_motor_t m3;
    cybergear_motor_t m4;

    cybergear_motor_t* _motors[NUM_MOTORS] = { &m1, &m2, &m3, &m4 };

    uint32_t alerts_triggered;
	twai_status_info_t twai_status;
	twai_message_t message;
	cybergear_status_t status;

    // Relay
    M5_4Relay relay;

    // Variables
    uint8_t _rx_can_id;

public:
    Motor()
    {
        // _motors[0] = &m1;
        // _motors[1] = &m2;
        // _motors[2] = &m3;
        // _motors[3] = &m4;
    }
    void init();
    void update();
    void print_motor(cybergear_motor_t* m);
    // void check_alerts();
    // void handle_rx_message(twai_message_t& message);
    void set_speed(float speed, int m_id);
    void enable(int m_id);
    void disable(int m_id);
    void unlock(int m_id);
    void lock(int m_id);
    void reset();
};