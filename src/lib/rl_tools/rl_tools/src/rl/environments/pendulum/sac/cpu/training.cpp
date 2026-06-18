#include "training.h"
int main(){
    run(0);
}

// Should take ~ 1.6s on M3 Pro in BECHMARK mode (tested @ 1118e19f904a26a9619fac7b1680643a0afcb695)
// Should take ~ 1.68s on M3 Pro in BECHMARK mode (tested @ 3ad4292ccab4e66f6c5487e19ef82de2fa88eacb)
// Should take ~ 1.938s on AMD Ryzen 9 7945HX in BECHMARK mode (tested @ 3ad4292ccab4e66f6c5487e19ef82de2fa88eacb) (MKL_NUM_THREADS=1 sudo -E nice -n-20 /home/jonas/rl-tools/cmake-build-release/src/rl/environments/pendulum/sac/cpu/rl_environments_pendulum_sac_benchmark)
