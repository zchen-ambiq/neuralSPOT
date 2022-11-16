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
    .eAIPowerMode = NS_MAXIMUM_PERF,
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

    ns_core_init();
  	ns_timer_init(&g_ns_tickTimer);

    // enables crypto
    // ns_itm_printf_enable();
    // ns_debug_printf_enable();
    ns_power_config(&ns_benchmark);
    ns_peripheral_button_init(&button_config);
    am_hal_interrupt_master_enable();
    ns_init_power_monitor_state();
	// ns_deep_sleep();
    init_streaming_model();
	// if (interpreter->arena_used_bytes() > kTensorArenaSize)
    // ns_lp_printf("arena used size %d\n", interpreter->arena_used_bytes());

	am_hal_timer_clear(0);
	// put some garbage in input tensor
	for (int i=0; i<1536; i++) {
		model_input->data.f[i] = i;
	}
	ns_set_power_monitor_state(NS_IDLE);
	ns_delay_us(1000000);
    // ns_lp_printf("Before: %d\n",ns_us_ticker_read(&g_ns_tickTimer));
	ns_set_power_monitor_state(NS_INFERING);
	for (int i=0; i<100; i++) 
    	try_streaming_model();
	ns_set_power_monitor_state(NS_IDLE);
    // ns_lp_printf("After: %d\n",ns_us_ticker_read(&g_ns_tickTimer));

	// print first few from output tensor
	// for (int i=0; i<10; i++) {
	// 	ns_lp_printf("%f\n", model_output->data.f[i]);
	// }

    while(1) {ns_deep_sleep();}
	// while(1);
}