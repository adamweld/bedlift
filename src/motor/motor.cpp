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
    /* request status */
    for(cybergear_motor_t* m : _motors){
        ESP_ERROR_CHECK_WITHOUT_ABORT(cybergear_request_status(m));
        ESP_ERROR_CHECK_WITHOUT_ABORT(cybergear_get_param(m, ADDR_ROTATION));
        ESP_ERROR_CHECK_WITHOUT_ABORT(cybergear_get_param(m, ADDR_MECH_POS));
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
        ESP_LOGE(TAG, "Alert: An error has occurred on the bus.Bus error count: %lu\n", twai_status.bus_error_count);
    }
    if (alerts_triggered & TWAI_ALERT_TX_FAILED)
    {
        ESP_LOGE(TAG, "Alert: The Transmission failed. buffered: %lu\terror: %lu\tfailed: %lu\n", twai_status.msgs_to_tx, twai_status.tx_error_counter, twai_status.tx_failed_count);
    }

    /* handle received messages */
    if (alerts_triggered & TWAI_ALERT_RX_DATA) 
    {
        while (twai_receive(&message, 0) == ESP_OK)
        {
            for (cybergear_motor_t* m : _motors) {
                esp_err_t response = cybergear_process_message(m, &message);
                if (response == ESP_ERR_NOT_FOUND) {
                    continue;
                }
                else if (response == ESP_OK) {
                    break;
                }
                else {
                    ESP_ERROR_CHECK_WITHOUT_ABORT(response);
                }
            }
        }
        /* get cybergear status*/
        for(cybergear_motor_t* m : _motors){
            cybergear_get_status(m, &status);
// // printf("M1 POS:%f V:%f T:%f temp:%f\t", status.position, status.speed, status.torque, status.temperature);
//             printf("M%d:%c V:%.2f T:%.2f\t",m->can_id, state_as_char(status.state), status.speed, status.torque);

//             /* get cybergear faults */
//             if(cybergear_has_faults(m))
//             {
//                 cybergear_fault_t faults;
//                 cybergear_get_faults(m, &faults);
//                 cybergear_print_faults(&faults);
//             }


            print_motor(m);
        }
            // printf("M1 POS:%f V:%f T:%f temp:%f\t", status.position, status.speed, status.torque, status.temperature);
        printf("\n");
    }
}

void Motor::print_motor(cybergear_motor_t* m)
{
    char fault_char = '*';
    if(cybergear_has_faults(m))
    {
        fault_char = 'X';
        cybergear_fault_t faults;
        cybergear_get_faults(m, &faults);
        
        ESP_LOGI(TAG, "Fault Overload: %s",btoa(faults->overload));
        ESP_LOGI(TAG, "Fault Uncalibrated: %s",btoa(faults->uncalibrated));
        ESP_LOGI(TAG, "Fault Over-Current Phase A: %s",btoa(faults->over_current_phase_a));
        ESP_LOGI(TAG, "Fault Over-Current Phase B: %s",btoa(faults->over_current_phase_b));
        ESP_LOGI(TAG, "Fault Over-Current Phase C: %s",btoa(faults->over_current_phase_c));
        ESP_LOGI(TAG, "Fault Over Voltage: %s",btoa(faults->over_voltage));
        ESP_LOGI(TAG, "Fault Under Voltage: %s",btoa(faults->under_voltage));
        ESP_LOGI(TAG, "Fault Driver-Chip: %s",btoa(faults->driver_chip));
        ESP_LOGI(TAG, "Fault Over-Temperature: %s",btoa(faults->over_temperature));
        ESP_LOGI(TAG, "Fault Magnetic Encoder: %s",btoa(faults->magnetic_code_failure));
        ESP_LOGI(TAG, "Fault Hall-Coded: %s",btoa(faults->hall_coded_faults));

    }   

    printf("M%d:%c%c V:%.2f R:%f T:%.2f\t",m->can_id, state_as_char(m->status.state),fault_char, m->status.speed, m->params.mech_pos, m->status.torque);

    /* get cybergear faults */
   
}


void Motor::set_speed(float speed, int m_id)
{
    if(m_id == 0) {
       for(cybergear_motor_t* m : _motors){ cybergear_set_speed(m, speed); }
    }
    else{
        cybergear_set_speed(_motors[m_id-1], speed);
    }
}

void Motor::enable(int m_id)
{
    if(m_id == 0) {
       for(cybergear_motor_t* m : _motors){ cybergear_enable(m); }
    }
    else{
        cybergear_enable(_motors[m_id-1]);
    }
}

void Motor::disable(int m_id)
{
    if(m_id == 0) {
       for(cybergear_motor_t* m : _motors){ cybergear_stop(m); }
    }
    else{
        cybergear_stop(_motors[m_id-1]);
    }
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