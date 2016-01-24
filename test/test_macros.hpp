/**
 * @file test_marcos.hpp
 *
 * Helps with cmake testing.
 *
 * @author James Goppert <james.goppert@gmail.com>
 */
#pragma once

#include <cstdio>

#define TEST(X) if(!(X)) { fprintf(stderr, "test failed on %s:%d\n", __FILE__, __LINE__); return -1;}
