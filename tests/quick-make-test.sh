#!/bin/bash
set -e

rm "build/examples/rpc_client_example/rpc_client.axf"           > /dev/null || true
rm "build/examples/rpc_server_example/rpc_server.axf"           > /dev/null || true
rm "build/examples/mpu_data_collection/mpu_data_collection.axf" > /dev/null || true
rm "build/examples/basic_tf_stub/basic_tf_stub.axf"             > /dev/null || true
rm "build/examples/rpc_client_example/rpc_client.bin"           > /dev/null || true
rm "build/examples/rpc_server_example/rpc_server.bin"           > /dev/null || true
rm "build/examples/mpu_data_collection/mpu_data_collection.bin" > /dev/null || true
rm "build/examples/basic_tf_stub/basic_tf_stub.bin"             > /dev/null || true
# rm "nest/build/basic_tf_stub.bin"                               || true

# make -f Makefile clean > /dev/null
make -f Makefile -j  > /dev/null
# make -f Makefile nestall > /dev/null > /dev/null
# cd nest
# make -f Makefile > /dev/null > /dev/null
# cd ..
test -f "build/examples/rpc_client_example/rpc_client.bin"           || { echo "FAIL rpc client doesn't exit"; exit 1; }
test -f "build/examples/rpc_server_example/rpc_server.bin"           || { echo "FAIL rpc server doesn't exit"; exit 1; }
test -f "build/examples/mpu_data_collection/mpu_data_collection.bin" || { echo "FAIL mpu data collection doesn't exist"; exit 1; }
test -f "build/examples/basic_tf_stub/basic_tf_stub.bin"             || { echo "FAIL basic TF stub doesn't exist"; exit 1; }
# test -f "nest/build/basic_tf_stub.bin"                               || { echo "FAIL Nests basic TF stub doesn't exist"; exit 1; }
