#include <stdio.h>

#include "edge-impulse-sdk/classifier/ei_run_classifier.h"
#include "ns_ambiqsuite_harness.h"
#include "ns_malloc.h"
#include "ns_timer.h"
#include "ns_i2c_register_driver.h"
#include "ns_mpu6050_i2c_driver.h"
#include "ns_core.h"

// Callback function declaration
static int get_signal_data(size_t offset, size_t length, float *out_ptr);

static float input_buf[600]; 

#define AXES 3
#define MPUREADLEN 32

ns_i2c_config_t i2cConfig = {
    .api = &ns_i2c_V1_0_0,
    .iom = 1};

uint32_t mpuAddr = MPU_I2CADDRESS_AD0_LOW;

ns_timer_config_t ei_tickTimer = {
    .api = &ns_timer_V1_0_0,
    .timer = NS_TIMER_COUNTER,
    .enableInterrupt = false,
};

static void read_1_second() {
    int t, axis;
    uint8_t buffer[MPUREADLEN];
    int16_t accelX, accelY, accelZ; //, gyroX, gyroY, gyroZ, temperature_regval;
    // float temperature = 0.0;
    float accelVals[AXES];
    // float gyroVals[AXES];

    for (t=0; t<200; t++) {
        read_sensors(buffer);
        mpu6050_get_accel_values(&i2cConfig, mpuAddr, &accelVals[0], &accelVals[1], &accelVals[2]);
    
        // // decode the buffer
        // accelX = buffer[0] << 8 | buffer[1];
        // accelY = buffer[2] << 8 | buffer[3];
        // accelZ = buffer[4] << 8 | buffer[5];

        // Not used by this model
        // temperature_regval = (buffer[6]<<8 | buffer[7]);
        // temperature = temperature_regval/340.0+36.53;

        // gyroX = buffer[8] << 8 | buffer[9];
        // gyroY = buffer[10] << 8 | buffer[11];
        // gyroZ = buffer[12] << 8 | buffer[13];

        // accelGravity(accelVals, accelX, accelY, accelZ, ACCEL_FS_16G);
        // gyroDegPerSec(gyroVals, gyroX, gyroY, gyroZ, GYRO_FS_500DPS);

        // Capture data in input_buf buffer
        for (axis = 0; axis<AXES; axis++) {
            // input_buf[axis+t*AXES] = accelVals[axis];
            input_buf[axis+t*AXES] = mpu6050_accel_to_gravity(accelVals[axis], ACCEL_FS_4G);
        }

        ns_delay_us(5000);
    }
}

uint32_t
mpu6050_init(ns_i2c_config_t *cfg, uint32_t devAddr) {
    if (mpu6050_device_reset(cfg, devAddr) ||
        mpu6050_set_clock_source(cfg, devAddr, CLOCK_GZ_PLL) ||
        mpu6050_set_lowpass_filter(cfg, devAddr, DLPF_044HZ) ||
        mpu6050_set_gyro_full_scale(cfg, devAddr, GYRO_FS_500DPS) ||
        mpu6050_set_accel_full_scale(cfg, devAddr, ACCEL_FS_4G) ||
        mpu6050_set_sample_rate(cfg, devAddr, 100) || mpu6050_set_sleep(cfg, devAddr, 0)) {
        return MPU6050_STATUS_ERROR;
    }
    return MPU6050_STATUS_SUCCESS;
}

int main(int argc, char **argv) {
    
    signal_t signal;            // Wrapper for raw input buffer
    ei_impulse_result_t result; // Used to store inference output
    EI_IMPULSE_ERROR res;       // Return code from inference
    void *mpuHandle;

    NS_TRY(ns_core_init(&ns_core_cfg), "Core init failed.\b");
    NS_TRY(ns_power_config(&ns_development_default), "Power Init Failed\n");
    ns_itm_printf_enable();
    ns_malloc_init();
    ns_interrupt_master_enable();
	NS_TRY(ns_timer_init(&ei_tickTimer), "Timer init failed.\n");
    ns_i2c_interface_init(1, MPU_I2CADDRESS_AD0_LOW, &mpuHandle);

    NS_TRY(ns_i2c_interface_init(&i2cConfig, 100000), "i2c Interface Init Failed.\n");
    mpu6050_init(&i2cConfig, mpuAddr);

    mpu6050_finish_init(mpuHandle);
    if(mpu6050_calibration()) {
        ns_printf("Calibration Failed!\n");
        return 1;
    }

    // Calculate the length of the buffer
    size_t buf_len = sizeof(input_buf) / sizeof(input_buf[0]);

    // Make sure that the length of the buffer matches expected input length
    if (buf_len != EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
        ns_printf("ERROR: The size of the input buffer is not correct.\r\n");
        ns_printf("Expected %d items, but got %d\r\n", 
                EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, 
                (int)buf_len);
        return 1;
    }

    // Assign callback function to fill buffer used for preprocessing/inference
    signal.total_length = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE;
    signal.get_data = &get_signal_data;

    while(1) {
        // Perform DSP pre-processing and inference
        // Collect a 1s sample

        read_1_second();

        res = run_classifier(&signal, &result, false);

        // Print return code and how long it took to perform inference
        ns_printf("run_classifier returned: %d\r\n", res);
        ns_printf("Timing: DSP %d ms, inference %d ms, anomaly %d ms\r\n", 
                result.timing.dsp, 
                result.timing.classification, 
                result.timing.anomaly);

        // Print the prediction results (object detection)
        #if EI_CLASSIFIER_OBJECT_DETECTION == 1
            ns_printf("Object detection bounding boxes:\r\n");
            for (uint32_t i = 0; i < EI_CLASSIFIER_OBJECT_DETECTION_COUNT; i++) {
                ei_impulse_result_bounding_box_t bb = result.bounding_boxes[i];
                if (bb.value == 0) {
                    continue;
                }
                ns_printf("  %s (%f) [ x: %u, y: %u, width: %u, height: %u ]\r\n", 
                        bb.label, 
                        bb.value, 
                        bb.x, 
                        bb.y, 
                        bb.width, 
                        bb.height);
            }

        // Print the prediction results (classification)
        #else
            ns_printf("Predictions:\r\n");
            for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
                ns_printf("  %s: ", ei_classifier_inferencing_categories[i]);
                ns_printf("%.5f\r\n", result.classification[i].value);
            }
        #endif

        // Print anomaly result (if it exists)
        #if EI_CLASSIFIER_HAS_ANOMALY == 1
            ns_printf("Anomaly prediction: %.3f\r\n", result.anomaly);
        #endif
    }
    return 0;
}

// Callback: fill a section of the out_ptr buffer when requested
static int get_signal_data(size_t offset, size_t length, float *out_ptr) {
    for (size_t i = 0; i < length; i++) {
        out_ptr[i] = (input_buf + offset)[i];
    }
    ns_printf("AccX[%d] = %f\n", offset, out_ptr[0]);
    return EIDSP_OK;
}