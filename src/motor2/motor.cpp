#include "motor.h"

void Motor::init()
{
    // intialize CAN, only done once
    // _motors[0].init_twai(RX_PIN, TX_PIN, /*serial_debug=*/true);
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
		(gpio_num_t) TX_PIN, 
		(gpio_num_t) RX_PIN, 
		TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    /* install TWAI driver */
    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_ERROR_CHECK(twai_start());
    ESP_ERROR_CHECK(twai_reconfigure_alerts(TWAI_ALERTS, NULL));

    relay.begin(Wire1,SDA_PIN,SCL_PIN);
    relay.SyncMode(true);
    relay.AllOff();

    /* initialize all motors */
	// cybergear_motor_t cybergear_motor;
	cybergear_init(&m1, MASTER_CAN_ID, M1_CAN_ID, POLLING_RATE_TICKS);
    cybergear_init(&m2, MASTER_CAN_ID, M2_CAN_ID, POLLING_RATE_TICKS);
    cybergear_init(&m3, MASTER_CAN_ID, M3_CAN_ID, POLLING_RATE_TICKS);
    cybergear_init(&m4, MASTER_CAN_ID, M4_CAN_ID, POLLING_RATE_TICKS);
	
	// cybergear_enable(&m1);
	// cybergear_set_position(&m1, 10.0); 

    // init motors one by one
    for(cybergear_motor_t* m : _motors){
        printf("Intializing Motor CAN ID: 0x%02X\n", m->can_id);

        ESP_ERROR_CHECK(cybergear_stop(m));
        cybergear_set_mode(m, CYBERGEAR_MODE_SPEED);
        cybergear_set_limit_speed(m, MOTOR_LIMIT_SPEED);
        cybergear_set_limit_current(m, MOTOR_LIMIT_CURRENT);
        cybergear_set_limit_torque(m, MOTOR_LIMIT_TORQUE);
    }
}

void Motor::update()
{
    // check_alerts();
    // for(XiaomiCyberGearDriver m : _motors){
    //     m.request_status();
    //     XiaomiCyberGearStatus cs = m.get_status();
    //     printf("M%dPOS:%f V:%f T:%f temp:%d\t", m.get_motor_can_id(), cs.position, cs.speed, cs.torque, cs.temperature);
    // }
    // printf("\n");


    
    /* request status */
    for(cybergear_motor_t* m : _motors){
        cybergear_request_status(m);
    }
    

    /* handle CAN alerts */ 
    twai_read_alerts(&alerts_triggered, POLLING_RATE_TICKS);
    twai_get_status_info(&twai_status);
    if (alerts_triggered & TWAI_ALERT_ERR_PASS)
    {
        ESP_LOGE(TAG, "Alert: TWAI controller has become error passive.");
    }
    if (alerts_triggered & TWAI_ALERT_BUS_ERROR)
    {
        ESP_LOGE(TAG, "Alert: A (Bit, Stuff, CRC, Form, ACK) error has occurred on the bus.");
        ESP_LOGE(TAG, "Bus error count: %lu\n", twai_status.bus_error_count);
    }
    if (alerts_triggered & TWAI_ALERT_TX_FAILED)
    {
        ESP_LOGE(TAG, "Alert: The Transmission failed.");
        ESP_LOGE(TAG, "TX buffered: %lu\t", twai_status.msgs_to_tx);
        ESP_LOGE(TAG, "TX error: %lu\t", twai_status.tx_error_counter);
        ESP_LOGE(TAG, "TX failed: %lu\n", twai_status.tx_failed_count);
    }
    /* handle received messages */
    if (alerts_triggered & TWAI_ALERT_RX_DATA) 
    {

        // uint8_t _rx_can_id = (message.identifier & 0xFF00) >> 8;

        // for(cybergear_motor_t* m : _motors) {
        //     if(m->can_id == _rx_can_id) {
        //         cybergear_process_message(m, &message);
        //         break; // Found the matching motor, no need to check further
        //     }
        // }
        while (twai_receive(&message, 0) == ESP_OK)
        {
            for (cybergear_motor_t* m : _motors) {
                esp_err_t response = cybergear_process_message(m, &message);
                // if (response == ESP_ERR_NOT_FOUND) {
                //     continue;
                // }
                // else if (response == ESP_OK) {
                //     break;
                // }
                // else {
                //     ESP_ERROR_CHECK_WITHOUT_ABORT(response);
                // }

            }
        }
        /* get cybergear status*/
        for(cybergear_motor_t* m : _motors){
            cybergear_get_status(m, &status);
            // printf("M1 POS:%f V:%f T:%f temp:%f\t", status.position, status.speed, status.torque, status.temperature);
            printf("M%d:%c V:%.2f T:%.2f\t",m->can_id, state_as_char(status.state), status.speed, status.torque);

            /* get cybergear faults */
            if(cybergear_has_faults(m))
            {
                cybergear_fault_t faults;
                cybergear_get_faults(m, &faults);
                cybergear_print_faults(&faults);
            }
        }
        printf("\n");
    }

    
    // m1.request_status();
    // XiaomiCyberGearStatus cs = m1.get_status();
    // printf("POS:%f V:%f T:%f temp:%d\n", cs.position, cs.speed, cs.torque, cs.temperature);
}


void Motor::set_speed(float speed, int m_id)
{
    // printf("Selected Motor %d set speed to %f\n",m_id, speed);
    // if ( m_id == 0 ) {
    //     for(XiaomiCyberGearDriver m : _motors){
    //         m.set_speed_ref(speed);
    //     }
    // } else {
    //     _motors[m_id-1].set_speed_ref(speed);
    // }

    cybergear_set_speed(_motors[m_id-1], speed);
}

void Motor::enable(int m_id)
{
    // if ( m_id == 0 ) {
    //     for(XiaomiCyberGearDriver m : _motors){
    //         m.enable_motor();
    //         m.set_position_ref(0.0);
    //     }
    // } else {
    //     _motors[m_id-1].enable_motor();
    //     _motors[m_id-1].set_position_ref(0.0);
    // }
    cybergear_enable(_motors[m_id-1]);
}

void Motor::disable(int m_id)
{
    // if ( m_id == 0 ) {
    //     for(XiaomiCyberGearDriver m : _motors){
    //         m.stop_motor();
    //     }
    // } else {
    //     _motors[m_id-1].stop_motor();
    // }
    cybergear_stop(_motors[m_id-1]);
}

void Motor::unlock(int m_id)
{
    relay.Write4Relay(3,true);
}

void Motor::lock(int m_id)
{
    relay.Write4Relay(3,false);
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