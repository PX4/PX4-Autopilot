// SPDX-License-Identifier: BSD-3-Clause

#include "DragonflyTurtleMapping.hpp"
#include <algorithm>
#include <cassert>
#include <iostream>
#include <limits>
using namespace dragonfly_mapping;
int main()
{
	Config cfg;
	cfg.motor[FL] = 3;
	cfg.motor[FR] = 1;
	cfg.motor[RL] = 2;
	cfg.motor[RR] = 4;
	int32_t f[16] {104, 103, 101, 102};
	auto r = resolve(cfg, f, 16, 0xf);
	assert(r.valid() && r.all == 0xf);
	assert(select(r, 1, 0) == 0x5 && select(r, -1, 0) == 0xa);
	assert(select(r, 0, 1) == 0x6 && select(r, 0, -1) == 0x9);
	assert(select(r, 1, 1) == 0x4 && select(r, -1, -1) == 0x8);
	assert(select(r, 0, 0) == 0 && select(r, 0.35f, 0) == 0);
	assert(select(r, std::numeric_limits<float>::quiet_NaN(), 0) == 0);
	assert(select(r, 0, std::numeric_limits<float>::infinity()) == 0);
	assert(select(r, 2, 0) == 0);
	unsigned tested = 0;
	// Every logical motor/corner permutation and every ordered four-slot placement on 12 outputs.
	int32_t motors[4] {1, 2, 3, 4};

	do {
		for (unsigned c = 0; c < 4; ++c) {
			cfg.motor[c] = motors[c];
		}

		for (unsigned a = 0; a < 12; ++a) {
			for (unsigned b = 0; b < 12; ++b) {
				if (b != a) {
					for (unsigned c = 0; c < 12; ++c) {
						if (c != a && c != b) {
							for (unsigned d = 0; d < 12; ++d) {
								if (d != a && d != b && d != c) {
									int32_t functions[12] {};
									unsigned slots[4] {a, b, c, d};
									uint32_t mask = 0;

									for (unsigned j = 0; j < 4; ++j) {
										functions[slots[j]] = 100 + cfg.motor[j];
										mask |= 1u << slots[j];
									}

									auto m = resolve(cfg, functions, 12, mask);
									assert(m.valid() && m.all == mask);
									assert(select(m, 1, 0) == ((1u << b) | (1u << d)));
									assert(select(m, -1, 0) == ((1u << a) | (1u << c)));
									assert(select(m, 0, 1) == ((1u << a) | (1u << b)));
									assert(select(m, 0, -1) == ((1u << c) | (1u << d)));
									assert(select(m, 1, 1) == (1u << b));
									++tested;
								}
							}
						}
					}
				}
			}
		}
	} while (std::next_permutation(motors, motors + 4));

	cfg.motor[0] = 1;
	cfg.motor[1] = 2;
	cfg.motor[2] = 3;
	cfg.motor[3] = 4;
	int32_t g[8] {101, 102, 103, 104};
	assert(resolve(cfg, g, 8, 0xf).valid());
	cfg.motor[0] = 0;
	assert(resolve(cfg, g, 8, 0xf).error == Error::UNCONFIGURED);
	cfg.motor[0] = 5;
	assert(resolve(cfg, g, 8, 0xf).error == Error::MOTOR_ID);
	cfg.motor[0] = 2;
	assert(resolve(cfg, g, 8, 0xf).error == Error::DUPLICATE_MOTOR);
	cfg.motor[0] = 1;
	g[4] = 105;
	assert(resolve(cfg, g, 8, 0x1f).error == Error::EXTRA_FUNCTION);
	g[4] = 201;
	assert(resolve(cfg, g, 8, 0x1f).error == Error::EXTRA_FUNCTION);
	g[4] = 101;
	assert(resolve(cfg, g, 8, 0x1f).error == Error::DUPLICATE_OUTPUT);
	g[4] = 0;
	assert(resolve(cfg, g, 8, 0x7).error == Error::INACTIVE_OUTPUT);
	g[3] = 0;
	assert(resolve(cfg, g, 8, 0xf).error == Error::MISSING_OUTPUT);
	assert(resolve(cfg, nullptr, 8, 0xf).error == Error::OUTPUT_COUNT);
	assert(resolve(cfg, g, 33, 0xf).error == Error::OUTPUT_COUNT);
	int32_t last[32] {};
	last[0] = 101;
	last[2] = 102;
	last[9] = 103;
	last[31] = 104;
	auto top = resolve(cfg, last, 32, 0x80000205u);
	assert(top.valid() && select(top, 1, 0) == 0x80000004u);
	assert(outputNumber(top.corner[RR]) == 32);
	assert(!same(r, top));
	std::cout << "PASS: " << tested << " motor/slot permutations; invalid maps and 32-bit boundaries\n";
}
