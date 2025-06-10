#include "../factory_test.h"

void FactoryTest::_motor_user(int m_id)
{
    printf("Selected Motor %d\n", m_id);

    _canvas->setFont(&fonts::Font0);

    char string_buffer[20];

    _enc_pos = 0;
    _enc.setPosition(_enc_pos);

    motor.enable(m_id);
    motor.unlock(m_id);

    while (1)
    {
        _canvas->fillScreen((uint32_t)0x6AB8A0);

        _canvas->fillRect(0, 0, 240, 25, (uint32_t)0x163820);
        _canvas->setTextSize(2);
        _canvas->setTextColor((uint32_t)0x6AB8A0);
        if (m_id == 0)
            snprintf(string_buffer, 20, "All Motors");
        else
            snprintf(string_buffer, 20, "Motor %d", m_id);

        _motor_speed = _enc_pos * .5;
        motor.set_speed(_motor_speed, m_id);
        _canvas->drawCenterString(string_buffer, _canvas->width() / 2, 5);

        _canvas->setTextSize(5);
        _canvas->setTextColor((uint32_t)0x163820);
        snprintf(string_buffer, 20, "%d", _enc_pos);
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
            motor.set_speed(_motor_speed, m_id);
            motor.lock(m_id);
            // here we should drive against pawl
            motor.disable(m_id);
            break;
        }
    }

    printf("quit motor test\n");
}