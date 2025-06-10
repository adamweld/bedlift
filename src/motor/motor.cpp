#include "motor.h"

void Motor::init()
{
    // intialize CAN, only done once
    _motors[0].init_twai(RX_PIN, TX_PIN, /*serial_debug=*/true);

    relay.begin(Wire1,SDA_PIN,SCL_PIN);
    relay.SyncMode(true);
    relay.AllOff();

    // init motors one by one
    for(XiaomiCyberGearDriver m : _motors){
        m.init_motor(MOTOR_MODE);
        m.set_limit_speed(MOTOR_LIMIT_SPEED);
        m.set_limit_current(MOTOR_LIMIT_CURRENT);
        m.set_position_ref(0.0);
    }
}

void Motor::update()
{
    check_alerts();
    for(XiaomiCyberGearDriver m : _motors){
        m.request_status();
        XiaomiCyberGearStatus cs = m.get_status();
        printf("M%dPOS:%f V:%f T:%f temp:%d\t", m.get_motor_can_id(), cs.position, cs.speed, cs.torque, cs.temperature);
    }
    printf("\n");
    
    // m1.request_status();
    // XiaomiCyberGearStatus cs = m1.get_status();
    // printf("POS:%f V:%f T:%f temp:%d\n", cs.position, cs.speed, cs.torque, cs.temperature);
}

void Motor::check_alerts(){
    // Check if alert happened
    uint32_t alerts_triggered;
    twai_read_alerts(&alerts_triggered, pdMS_TO_TICKS(POLLING_RATE_MS));
    twai_status_info_t twai_status;
    twai_get_status_info(&twai_status);

    // Handle alerts
    if (alerts_triggered & TWAI_ALERT_ERR_PASS) {
        printf("Alert: TWAI controller has become error passive.\n");
    }
    if (alerts_triggered & TWAI_ALERT_BUS_ERROR) {
        printf("Alert: A (Bit, Stuff, CRC, Form, ACK) error has occurred on the bus.\n");
        printf("Bus error count: %d\n", twai_status.bus_error_count);
        printf("resetting motors");
        reset();
    }
    if (alerts_triggered & TWAI_ALERT_TX_FAILED) {
        printf("Alert: The Transmission failed.\n");
        printf("TX buffered: %d\t", twai_status.msgs_to_tx);
        printf("TX error: %d\t", twai_status.tx_error_counter);
        printf("TX failed: %d\n", twai_status.tx_failed_count);
    }

    // if (alerts_triggered & TWAI_ALERT_TX_SUCCESS) {
    //     printf("Alert: The Transmission was successful.\n");
    //     printf("TX buffered: %d\t", twai_status.msgs_to_tx);
    // }

    // Check if message is received
    if (alerts_triggered & TWAI_ALERT_RX_DATA) {
        twai_message_t message;
        while (twai_receive(&message, 0) == ESP_OK) {
            handle_rx_message(message);
        }
    }
}

void Motor::handle_rx_message(twai_message_t& message) {
    _rx_can_id = ( message.identifier & 0xFF00) >> 8;
    printf("Received CAN ID: 0x%02X\n", _rx_can_id);

    for(int ii = 0; ii < NUM_MOTORS; ii++){
        if (_rx_can_id == _motors[ii].get_motor_can_id()){
            // printf("processing message on motor %d\n", _motors[ii].get_motor_can_id());
            _motors[ii].process_message(message);
        }
    }
  // print received message
  // Serial.printf("ID: %x\nByte:", message.identifier);
  // if (!(message.rtr)) {
  //   for (int i = 0; i < message.data_length_code; i++) {
  //     Serial.printf(" %d = %02x,", i, message.data[i]);
  //   }
  //   Serial.println("");
  // }
}

void Motor::set_speed(float speed, int m_id)
{
    // printf("Selected Motor %d set speed to %f\n",m_id, speed);
    if ( m_id == 0 ) {
        for(XiaomiCyberGearDriver m : _motors){
            m.set_speed_ref(speed);
        }
    } else {
        _motors[m_id-1].set_speed_ref(speed);
    }
}

void Motor::enable(int m_id)
{
    if ( m_id == 0 ) {
        for(XiaomiCyberGearDriver m : _motors){
            m.enable_motor();
            m.set_position_ref(0.0);
        }
    } else {
        _motors[m_id-1].enable_motor();
        _motors[m_id-1].set_position_ref(0.0);
    }
}

void Motor::disable(int m_id)
{
    if ( m_id == 0 ) {
        for(XiaomiCyberGearDriver m : _motors){
            m.stop_motor();
        }
    } else {
        _motors[m_id-1].stop_motor();
    }
}

void Motor::unlock(int m_id)
{
    if ( m_id == 0 ) {
        relay.Write4Relay(0,true);
        relay.Write4Relay(1,true);
        relay.Write4Relay(2,true);
        relay.Write4Relay(3,true);
    } else {
        relay.Write4Relay(m_id-1,true);
    }
}

void Motor::lock(int m_id)
{
    if ( m_id == 0 ) {
        relay.AllOff();
    } else {
        relay.Write4Relay(m_id-1,false);
    }
}

void Motor::reset()
{
    // _motors[0].init_motor
    // Install TWAI driver
    if (twai_driver_uninstall() == ESP_OK) {
        printf("Uninstalled TWAI driver\n");
    } else {
        printf("Failed to uninstall TWAI driver\n");
    }
    init();
}