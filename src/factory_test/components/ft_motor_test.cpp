#include "../factory_test.h"

void FactoryTest::_motor_speed_control(const std::vector<int>& motor_ids)
{
    printf("Selected Motors: ");
    for(int i = 0; i < motor_ids.size(); i++) {
        printf("%d", motor_ids[i]);
        if(i < motor_ids.size() - 1) printf(", ");
    }
    printf("\n");

    _canvas->setFont(&fonts::Font0);

    char string_buffer[30];

    _enc_pos = 0;
    _enc.setPosition(_enc_pos);

    // motor.enable(motor_ids);
    motor.unlock(motor_ids);

    while (1)
    {
        _canvas->fillScreen((uint32_t)0x6AB8A0);

        _canvas->fillRect(0, 0, 240, 25, (uint32_t)0x163820);
        _canvas->setTextSize(2);
        _canvas->setTextColor((uint32_t)0x6AB8A0);
        
        // Generate display string based on motor IDs
        if (motor_ids.size() == 4) {
            snprintf(string_buffer, 30, "All Motors");
        } else if (motor_ids.size() == 1) {
            snprintf(string_buffer, 30, "Motor %d", motor_ids[0]);
        } else {
            snprintf(string_buffer, 30, "Motors ");
            for(int i = 0; i < motor_ids.size(); i++) {
                char temp[5];
                snprintf(temp, 5, "%d", motor_ids[i]);
                strcat(string_buffer, temp);
                if(i < motor_ids.size() - 1) strcat(string_buffer, ",");
            }
        }

        _motor_speed = _enc_pos * .5;
        motor.set_speed(_motor_speed, motor_ids);
        _canvas->drawCenterString(string_buffer, _canvas->width() / 2, 5);

        _canvas->setTextSize(5);
        _canvas->setTextColor((uint32_t)0x163820);
        snprintf(string_buffer, 30, "%d", _enc_pos);
        _canvas->drawCenterString(string_buffer, _canvas->width() / 2, 55);

        _canvas_update();

        if (millis() - _motor_time_count > 500)
        {
            motor.update();
            _motor_time_count = millis();
        }

        _check_encoder();
        if (_check_next())
        {
            _motor_speed = 0.0;
            motor.set_speed(_motor_speed, motor_ids);
            motor.lock(motor_ids);
            // here we should drive against pawl
            motor.disable(motor_ids);
            break;
        }
    }

    printf("quit motor speed control\n");
}