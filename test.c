#include "test.h"
#include "follow.h"
#include "oled_hardware_i2c.h"
#include "stdio.h"

void Test_GraySensor_Show(void)
{
    uint8_t buf[18];

    IRDM_read_sensors();

    OLED_ShowString(0, 6, (uint8_t *)"f1f2f3f4f5f6f7f8", 8);

    for (int i = 0; i < 8; i++) {
        buf[i] = IRDM_get_sensor_state(i) ? '1' : '0';
    }
    buf[8] = '\0';

    OLED_ShowString(0, 7, buf, 8);
}