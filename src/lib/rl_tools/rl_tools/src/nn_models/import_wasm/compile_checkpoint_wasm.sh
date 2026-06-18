
/opt/homebrew/opt/llvm/bin/clang++ -I../../../include -std=c++17 --target=wasm32 -nostdlib -Wl,--no-entry \
-Wl,--export=batch_size \
-Wl,--export=input_dim \
-Wl,--export=output_dim \
-Wl,--export=set_input \
-Wl,--export=get_output \
-Wl,--export=evaluate \
-Wl,--export=example_batch_size \
-Wl,--export=get_example_input \
-Wl,--export=get_example_output \
-Wl,--export=test \
-o interface.wasm interface.cpp