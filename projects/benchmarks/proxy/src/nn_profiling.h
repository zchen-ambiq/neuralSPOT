#ifndef NN_PROFILING_H_
#define NN_PROFILING_H_

void init_streaming_model();
void try_streaming_model();
bool do_inference();
bool init_model(const unsigned char* model_buf);

#endif //NN_PROFILING_H_
