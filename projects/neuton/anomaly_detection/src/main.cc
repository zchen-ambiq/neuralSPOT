#include <stdio.h>

#include "ns_ambiqsuite_harness.h"
// #include "ns_malloc.h"
#include "ns_timer.h"
#include "ns_i2c_register_driver.h"
#include "ns_mpu6050_i2c_driver.h"
#include "ns_core.h"
#include "ns_peripherals_button.h"
#include "ns_peripherals_power.h"

#include "neuton/neuton.h"

int
main(void) {
    ns_core_config_t ns_core_cfg = {.api = &ns_core_V1_0_0};

    NS_TRY(ns_core_init(&ns_core_cfg), "Core init failed.\b");
    NS_TRY(ns_power_config(&ns_development_default), "Power Init Failed\n");
    ns_itm_printf_enable();
    ns_interrupt_master_enable();

    neuton_nn_setup();

    
}


