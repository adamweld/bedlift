#include "motor.h"

void Motor::init()
{
    printf("=== Motor System Initialization ===\n");
    
    // intialize CAN, only done once
    printf("Initializing CAN/TWAI transceiver...\n");
    printf("TX Pin: %d, RX Pin: %d\n", TX_PIN, RX_PIN);
    
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
		(gpio_num_t) TX_PIN, 
		(gpio_num_t) RX_PIN, 
		TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    /* install TWAI driver */
    esp_err_t err = twai_driver_install(&g_config, &t_config, &f_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "TWAI driver install failed: %s", esp_err_to_name(err));
        printf("ERROR: CAN transceiver initialization failed!\n");
        return;
    }
    printf("TWAI driver installed successfully\n");

    err = twai_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "TWAI start failed: %s", esp_err_to_name(err));
        printf("ERROR: CAN transceiver start failed!\n");
        return;
    }
    printf("TWAI started successfully\n");

    err = twai_reconfigure_alerts(TWAI_ALERTS, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "TWAI alerts config failed: %s", esp_err_to_name(err));
        printf("WARNING: CAN alerts configuration failed\n");
    }

    printf("Initializing relay controller...\n");
    printf("SDA Pin: %d, SCL Pin: %d\n", SDA_PIN, SCL_PIN);
    relay.begin(Wire1,SDA_PIN,SCL_PIN);
    relay.SyncMode(true);
    relay.AllOff();
    printf("Relay controller initialized\n");

    /* initialize all motors */
    printf("Initializing motor structures...\n");
    printf("Master CAN ID: 0x%02X, Motor CAN IDs: 0x%02X, 0x%02X, 0x%02X, 0x%02X\n", 
           MASTER_CAN_ID, M1_CAN_ID, M2_CAN_ID, M3_CAN_ID, M4_CAN_ID);
    
	cybergear_init(&m1, MASTER_CAN_ID, M1_CAN_ID, POLLING_RATE_TICKS);
    cybergear_init(&m2, MASTER_CAN_ID, M2_CAN_ID, POLLING_RATE_TICKS);
    cybergear_init(&m3, MASTER_CAN_ID, M3_CAN_ID, POLLING_RATE_TICKS);
    cybergear_init(&m4, MASTER_CAN_ID, M4_CAN_ID, POLLING_RATE_TICKS);
    printf("Motor structures initialized\n");

    // Configure each motor
    printf("Configuring motor parameters...\n");
    for(int i = 0; i < NUM_MOTORS; i++){
        cybergear_motor_t* m = _motors[i];
        printf("Configuring Motor %d (CAN ID: 0x%02X)...\n", i+1, m->can_id);
        
        // Test motor presence by requesting status and waiting for response
        printf("Testing Motor %d presence...\n", i+1);
        esp_err_t err = cybergear_request_status(m);
        if (err != ESP_OK) {
            printf("ERROR: Motor %d status request failed: %s\n", i+1, esp_err_to_name(err));
            printf("Motor %d configuration FAILED - CAN transmission error\n", i+1);
            continue;
        }
        
        // Wait for response with timeout
        bool motor_responded = false;
        uint32_t start_time = millis();
        const uint32_t response_timeout_ms = 500;
        
        while(millis() - start_time < response_timeout_ms) {
            twai_message_t message;
            if(twai_receive(&message, pdMS_TO_TICKS(10)) == ESP_OK) {
                if(cybergear_process_message(m, &message) == ESP_OK) {
                    motor_responded = true;
                    break;
                }
            }
        }
        
        if (!motor_responded) {
            printf("ERROR: Motor %d did not respond to status request\n", i+1);
            printf("Motor %d configuration FAILED - No response (motor unplugged?)\n", i+1);
            continue;
        }
        
        printf("Motor %d responded, proceeding with configuration...\n", i+1);
        bool motor_ok = true;

        err = cybergear_set_mech_position_to_zero(m);
        err = cybergear_stop(m);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Motor %d stop failed: %s", i+1, esp_err_to_name(err));
            printf("WARNING: Motor %d stop command failed\n", i+1);
            motor_ok = false;
        }

        err = cybergear_set_mode(m, CYBERGEAR_MODE_SPEED);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Motor %d mode set failed: %s", i+1, esp_err_to_name(err));
            printf("ERROR: Motor %d mode configuration failed\n", i+1);
            motor_ok = false;
        }

        err = cybergear_set_limit_speed(m, MOTOR_LIMIT_SPEED);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Motor %d speed limit failed: %s", i+1, esp_err_to_name(err));
            printf("ERROR: Motor %d speed limit failed\n", i+1);
            motor_ok = false;
        }

        err = cybergear_set_limit_current(m, MOTOR_LIMIT_CURRENT);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Motor %d current limit failed: %s", i+1, esp_err_to_name(err));
            printf("ERROR: Motor %d current limit failed\n", i+1);
            motor_ok = false;
        }

        err = cybergear_set_limit_torque(m, MOTOR_LIMIT_TORQUE);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Motor %d torque limit failed: %s", i+1, esp_err_to_name(err));
            printf("ERROR: Motor %d torque limit failed\n", i+1);
            motor_ok = false;
        }

        if (motor_ok) {
            printf("Motor %d configured successfully\n", i+1);
        } else {
            printf("Motor %d configuration FAILED\n", i+1);
        }
    }
    
    printf("=== Motor System Initialization Complete ===\n");
}

void Motor::idle_update()
{
    /* request parameters while idle */
    for(cybergear_motor_t* m : _motors){
        cybergear_request_status(m);
        cybergear_request_param(m, ADDR_ROTATION);
        cybergear_request_param(m, ADDR_MECH_POS);
        cybergear_request_param(m, ADDR_VBUS);
    }

    /* handle CAN alerts */ 
    twai_read_alerts(&alerts_triggered, pdMS_TO_TICKS(10));
    twai_get_status_info(&twai_status);
    if (alerts_triggered & TWAI_ALERT_ERR_PASS)
    {
        ESP_LOGE(TAG, "Alert: TWAI controller has become error passive.");
        printf("CAN ERROR: Controller entered error passive state\n");
    }
    if (alerts_triggered & TWAI_ALERT_BUS_ERROR)
    {
        ESP_LOGE(TAG, "Alert: An error has occurred on the bus.Bus error count: %lu\n", twai_status.bus_error_count);
        printf("CAN ERROR: Bus error detected, count: %lu\n", twai_status.bus_error_count);
    }
    if (alerts_triggered & TWAI_ALERT_TX_FAILED)
    {
        ESP_LOGE(TAG, "Alert: The Transmission failed. buffered: %lu\terror: %lu\tfailed: %lu\n", twai_status.msgs_to_tx, twai_status.tx_error_counter, twai_status.tx_failed_count);
        printf("CAN ERROR: Transmission failed - buffered: %lu, errors: %lu, failed: %lu\n", 
               twai_status.msgs_to_tx, twai_status.tx_error_counter, twai_status.tx_failed_count);
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
    }
    
    /* print parameter values for each motor */
    printf("IDLE PARAMS: ");
    for(cybergear_motor_t* m : _motors){
        printf("M%d[R:%d P:%.2f V:%.1f] ", 
               m->can_id, 
               m->params.rotation, 
               m->params.mech_pos, 
               m->params.vbus);
    }
    printf("\n");
}

void Motor::update()
{
    /* request status */

    // this doesn't seem to work when running
    // for(cybergear_motor_t* m : _motors){
    //     // ESP_ERROR_CHECK_WITHOUT_ABORT(cybergear_request_status(m));
    //     // ESP_ERROR_CHECK_WITHOUT_ABORT(cybergear_get_param(m, ADDR_ROTATION));
    //     // ESP_ERROR_CHECK_WITHOUT_ABORT(cybergear_get_param(m, ADDR_MECH_POS));
    //     // cybergear_request_status(m);
    //     cybergear_request_param(m, ADDR_ROTATION);
    //     cybergear_request_param(m, ADDR_MECH_POS);
    //     cybergear_request_param(m, ADDR_VBUS);
// 
    // }

    /* handle CAN alerts */ 
    twai_read_alerts(&alerts_triggered, pdMS_TO_TICKS(10));
    twai_get_status_info(&twai_status);
    if (alerts_triggered & TWAI_ALERT_ERR_PASS)
    {
        ESP_LOGE(TAG, "Alert: TWAI controller has become error passive.");
        printf("CAN ERROR: Controller entered error passive state\n");
    }
    if (alerts_triggered & TWAI_ALERT_BUS_ERROR)
    {
        ESP_LOGE(TAG, "Alert: An error has occurred on the bus.Bus error count: %lu\n", twai_status.bus_error_count);
        printf("CAN ERROR: Bus error detected, count: %lu\n", twai_status.bus_error_count);
    }
    if (alerts_triggered & TWAI_ALERT_TX_FAILED)
    {
        ESP_LOGE(TAG, "Alert: The Transmission failed. buffered: %lu\terror: %lu\tfailed: %lu\n", twai_status.msgs_to_tx, twai_status.tx_error_counter, twai_status.tx_failed_count);
        printf("CAN ERROR: Transmission failed - buffered: %lu, errors: %lu, failed: %lu\n", 
               twai_status.msgs_to_tx, twai_status.tx_error_counter, twai_status.tx_failed_count);
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
        printf("VBUS:%.2f\n", _motors[0]->params.vbus);
            // printf("M1 POS:%f V:%f T:%f temp:%f\t", status.position, status.speed, status.torque, status.temperature);
        // printf("\n");
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

    printf("M%d:%c%c V:%.2f P:%.2f R:%d T:%.2f\t",m->can_id, state_as_char(m->status.state),fault_char, m->status.speed, m->status.position, m->params.rotation, m->status.torque);

    /* get cybergear faults */
   
}

void Motor::set_speed(float speed, const std::vector<int>& m_ids)
{
    for(int m_id : m_ids) {
        if(m_id >= 1 && m_id <= NUM_MOTORS) {
            cybergear_set_speed(_motors[m_id-1], speed);
        }
    }
}

void Motor::enable(const std::vector<int>& m_ids)
{
    for(int m_id : m_ids) {
        if(m_id >= 1 && m_id <= NUM_MOTORS) {
            cybergear_enable(_motors[m_id-1]);
        }
    }
}

void Motor::disable(const std::vector<int>& m_ids)
{
    for(int m_id : m_ids) {
        if(m_id >= 1 && m_id <= NUM_MOTORS) {
            cybergear_stop(_motors[m_id-1]);
        }
    }
}

void Motor::unlock(const std::vector<int>& m_ids)
{
    // Disengage relay (shared between all motors)
    relay.Write4Relay(3, true);

    const float unlock_speed = -3.0f;
    const uint32_t unlock_duration_ms = 350;
    
    // Enable motors
    for(int m_id : m_ids) {
        if(m_id >= 1 && m_id <= NUM_MOTORS) {
            cybergear_enable(_motors[m_id-1]);
            cybergear_set_speed(_motors[m_id-1], unlock_speed);
        }
    }
    delay(unlock_duration_ms);
}

void Motor::lock(const std::vector<int>& m_ids)
{
    // Engage relay (shared between all motors)
    relay.Write4Relay(3, false);
    
    // Drive motors at 1 rad/s for 400ms to softly engage against solenoid
    const float lock_speed = 2.0f;
    const uint32_t lock_duration_ms = 600;
    
    for(int m_id : m_ids) {
        if(m_id >= 1 && m_id <= NUM_MOTORS) {
            cybergear_set_speed(_motors[m_id-1], lock_speed);
        }
    }
    
    // Wait for motors to drive against the lock
    delay(lock_duration_ms);
    
    // Disable motors
    for(int m_id : m_ids) {
        if(m_id >= 1 && m_id <= NUM_MOTORS) {
            cybergear_stop(_motors[m_id-1]);
        }
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