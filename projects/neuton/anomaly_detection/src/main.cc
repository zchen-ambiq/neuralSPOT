#include <stdio.h>

#include "ns_ambiqsuite_harness.h"
// #include "ns_malloc.h"
#include "ns_timer.h"
#include "ns_i2c_register_driver.h"
#include "ns_mpu6050_i2c_driver.h"
#include "ns_core.h"
#include "ns_peripherals_button.h"
#include "ns_peripherals_power.h"
#include "testdata.h"
#include "neuton/neuton.h"

int
main(void) {

    ns_core_config_t ns_core_cfg = {.api = &ns_core_V1_0_0};

    NS_TRY(ns_core_init(&ns_core_cfg), "Core init failed.\b");
    NS_TRY(ns_power_config(&ns_development_default), "Power Init Failed\n");
    ns_itm_printf_enable();
    ns_interrupt_master_enable();
    ns_lp_printf("hello world");
    neuton_input_t raw_inputs[5000];
    for(int i = 0; i < 5000; i++) {
        raw_inputs[i] = testData[1][i];
    }
    neuton_nn_setup();
    neuton_inference_input_t* p_input;
    p_input = neuton_nn_feed_inputs(raw_inputs, neuton_nn_uniq_inputs_num() * neuton_nn_input_window_size());

    /** Run inference */
    if (p_input)
    {
        neuton_u16_t predicted_target;
        const neuton_output_t* probabilities;
        neuton_i16_t outputs_num = neuton_nn_run_inference(p_input, &predicted_target, &probabilities);

        if (outputs_num > 0)
        {
            printf("Predicted target %d with probability %f\r\n", predicted_target, probabilities[predicted_target]);

            printf("All probabilities:\r\n");
            for (size_t i = 0; i < outputs_num; i++)
                printf("%f,", probabilities[i]);
        }
    }
}


