#include "ns_core.h"

#include "basic_peripherals.h"
#ifdef RPC_ENABLED
#include "basic_rpc_client.h"
#endif

/// NeuralSPOT Includes
#include "ns_ambiqsuite_harness.h"
#include "ns_peripherals_power.h"
#include "ns_usb.h"
#include "ns_energy_monitor.h"
#include "ns_cache_profile.h"
#include "ns_power_profile.h"

#define TF_LITE_STATIC_MEMORY 1
#include "nn_profiling.h"
#include "tensorflow/lite/micro/all_ops_resolver.h"
#include "tensorflow/lite/micro/kernels/micro_ops.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/recording_micro_interpreter.h"
#include "tensorflow/lite/micro/system_setup.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/micro/micro_allocator.h"

//#include "conv_layers.h"
#include "streaming_model.h"


static constexpr int kTensorArenaSize = 1024 * 75;
alignas(16) static uint8_t tensor_arena[kTensorArenaSize];
static constexpr int kVarArenaSize = 1024 * 10;
alignas(16) static uint8_t var_arena[kVarArenaSize];

static const tflite::Model* model = nullptr;
static tflite::ErrorReporter* er = nullptr;
static tflite::MicroInterpreter* interpreter = nullptr;

static tflite::MicroAllocator* var_allocator = nullptr;
static tflite::MicroResourceVariables* resource_variables = nullptr;
volatile TfLiteTensor *model_input = nullptr;
volatile TfLiteTensor *model_output = nullptr;

bool init_model(const unsigned char* model_buf) {
	static tflite::MicroErrorReporter micro_error_reporter;
	er = &micro_error_reporter;
	static tflite::AllOpsResolver resolver;
	tflite::InitializeTarget();
	model = tflite::GetModel(model_buf);

	var_allocator = tflite::MicroAllocator::Create(var_arena,kVarArenaSize, er);
	//appears to be 14 resourcevariables
	resource_variables = tflite::MicroResourceVariables::Create(var_allocator, 15);
	static tflite::RecordingMicroInterpreter static_interpreter(tflite::RecordingMicroInterpreter(model, 
																		resolver, tensor_arena, 
																		kTensorArenaSize, er, resource_variables));
	interpreter = &static_interpreter;

	// Allocate memory from the tensor_arena for the model's tensors.
	TfLiteStatus allocate_status = interpreter->AllocateTensors();
	if (allocate_status != kTfLiteOk) {
		printf("AllocateTensors() failed\n");
		return false;
	}
    model_input  = interpreter->input(0);
    model_output = interpreter->output(0);
	return true;
}

bool do_inference() {
	if(interpreter == nullptr) {
		printf("model not initialized\n");
		return false;
	}
	
	TfLiteStatus invoke_status = interpreter->Invoke();
	if (invoke_status != kTfLiteOk) {
		printf("Invoke failed\n");
			return false;
	}

  return true;
}

void try_model(const unsigned char* model_buf) {

//	init_model(model_buf);
	do_inference();
	static_cast<tflite::RecordingMicroInterpreter *>(interpreter)
	      ->GetMicroAllocator()
	      .PrintAllocations();
}
void init_streaming_model() {
	init_model(streaming_tflite);
}

void try_streaming_model() {
   try_model(streaming_tflite);
}

const ns_power_config_t ns_benchmark = {
    // .eAIPowerMode = NS_MAXIMUM_PERF,
    .eAIPowerMode = NS_MINIMUM_PERF,
    .bNeedAudAdc = false,
    .bNeedSharedSRAM = false,
    .bNeedCrypto = false,
    .bNeedBluetooth = false,
    .bNeedUSB = false,
    .bNeedIOM = false,
    .bNeedAlternativeUART = false,
    .b128kTCM = true,
    .bEnableTempCo = false,
    .bNeedITM = false
};

ns_timer_config_t g_ns_tickTimer = {
    .prefix = {0},
    .timer = NS_TIMER_COUNTER,
    .enableInterrupt = false,
};

int main(void) {
	// ns_cache_config_t cc;
	// ns_cache_dump_t start;
	// ns_cache_dump_t end;

    ns_core_init();
  	// ns_timer_init(&g_ns_tickTimer);
    MCUCTRL->ADCBATTLOAD_b.BATTLOAD = MCUCTRL_ADCBATTLOAD_BATTLOAD_DIS;


    // enables crypto
    // ns_itm_printf_enable();
    // ns_debug_printf_enable();
    ns_power_config(&ns_benchmark);
    // am_hal_pwrctrl_control(AM_HAL_PWRCTRL_CONTROL_DIS_PERIPHS_ALL, 0);
    // MCUCTRL->PWRSW0_b.PWRSWVDDMDSP0DYNSEL = 0;    // Bit 18
    // MCUCTRL->PWRSW0_b.PWRSWVDDMDSP1DYNSEL = 0;    // Bit 21
    // MCUCTRL->PWRSW0_b.PWRSWVDDMLDYNSEL    = 0;    // Bit 24
	//  MCUCTRL->MRAMPWRCTRL_b.MRAMPWRCTRL = 1;
        am_hal_pwrctrl_mcu_memory_config_t McuMemCfg =
        {
            .eCacheCfg    = AM_HAL_PWRCTRL_CACHE_ALL,
            .bRetainCache = false,
            .eDTCMCfg     = AM_HAL_PWRCTRL_DTCM_128K,
            .eRetainDTCM  = AM_HAL_PWRCTRL_DTCM_128K,
            .bEnableNVM0  = true,
            .bRetainNVM0  = false
        };

        am_hal_pwrctrl_mcu_memory_config(&McuMemCfg);
    am_hal_pwrctrl_dsp_memory_config_t sExtSRAMMemCfg =
    {
        .bEnableICache      = false,
        .bRetainCache       = false,
        .bEnableRAM         = false,
        .bActiveRAM         = false,
        .bRetainRAM         = false
    };

    if (am_hal_pwrctrl_dsp_memory_config(AM_HAL_DSP0, &sExtSRAMMemCfg) != 0)
    // ||am_hal_pwrctrl_dsp_memory_config(AM_HAL_DSP1, &sExtSRAMMemCfg) != 0)
    {
        am_util_stdio_printf("DSP memory init error.\n");
    }
    MCUCTRL->ADCBATTLOAD_b.BATTLOAD = MCUCTRL_ADCBATTLOAD_BATTLOAD_DIS;
    MCUCTRL->AUDADCPWRCTRL_b.AUDADCPWRCTRLSWE = 1;
    MCUCTRL->AUDADCPWRCTRL_b.AUDADCAPSEN = 0;
    MCUCTRL->AUDADCPWRCTRL_b.AUDADCBPSEN = 0;
    MCUCTRL->AUDADCPWRCTRL_b.AUDBGTPEN = 0;
    MCUCTRL->AUDADCPWRCTRL_b.AUDREFBUFPEN = 0;
    MCUCTRL->AUDADCPWRCTRL_b.AUDREFKEEPPEN = 0;
    MCUCTRL->AUDADCPWRCTRL_b.AUDADCINBUFEN = 0;
    // MCUCTRL->AUDADCPWRCTRL_b.AUDREFKEEPPEN = 0;
    // MCUCTRL->AUDADCPWRCTRL_b.AUDREFKEEPPEN = 0;



	// cc.enable = true;
	// ns_cache_profiler_init(&cc);

	// ns_pp_snapshot(false,0);
    // ns_peripheral_button_init(&button_config);
    // am_hal_interrupt_master_enable();
    ns_init_power_monitor_state();
	// ns_deep_sleep();
    init_streaming_model();
	// if (interpreter->arena_used_bytes() > kTensorArenaSize)
    // ns_lp_printf("arena used size %d\n", interpreter->arena_used_bytes());

	// am_hal_timer_clear(0);
	// put some garbage in input tensor
	for (int i=0; i<1536; i++) {
		model_input->data.f[i] = i;
	}
	ns_set_power_monitor_state(NS_IDLE);


	// ns_delay_us(1000000);
    // ns_lp_printf("Before: %d\n",ns_us_ticker_read(&g_ns_tickTimer));
	// ns_capture_cache_stats(&start);
	ns_set_power_monitor_state(NS_INFERING);
	for (int i=0; i<100; i++) 
    	try_streaming_model();
	ns_set_power_monitor_state(NS_IDLE);
	// ns_capture_cache_stats(&end);

	// ns_print_cache_stats_delta(&start, &end);

    // ns_lp_printf("After: %d\n",ns_us_ticker_read(&g_ns_tickTimer));

	// print first few from output tensor
	// for (int i=0; i<10; i++) {
	// 	ns_lp_printf("%f\n", model_output->data.f[i]);
	// }

    while(1) {am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);}
	// while(1);
}