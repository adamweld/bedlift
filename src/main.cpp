/**
 * @file main.cpp
 * @author Forairaaaaa
 * @brief
 * @version 0.1
 * @date 2023-06-06
 *
 * @copyright Copyright (c) 2023
 *
 */
#include "factory_test/factory_test.h"

static FactoryTest ft;

void view_create(FactoryTest* ft);
void view_update();

void setup()
{
    delay(5000);
    printf("\n\n=== BEDLIFT SYSTEM STARTING ===\n");
    printf("Firmware Version: %s\n", FW_VERISON);
    printf("ESP32 Ready, initializing factory test...\n");
    
    ft.init();
    printf("Factory test initialized\n");
    
    view_create(&ft);
    printf("View created, entering main loop\n");
    printf("=== STARTUP COMPLETE ===\n\n");
}

void loop() {
    view_update();
}
