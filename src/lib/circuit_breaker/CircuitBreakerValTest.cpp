/****************************************************************************
 *
 *   Copyright (C) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

/**
 * Unit tests for circuit_breaker_enabled_by_val magic match.
 * make tests TESTFILTER=CircuitBreakerVal
 */

#include <gtest/gtest.h>
#include "circuit_breaker.h"

TEST(CircuitBreakerVal, MatchesMagic)
{
	EXPECT_TRUE(circuit_breaker_enabled_by_val(CBRK_BUZZER_KEY, CBRK_BUZZER_KEY));
	EXPECT_TRUE(circuit_breaker_enabled_by_val(CBRK_IO_SAFETY_KEY, CBRK_IO_SAFETY_KEY));
	EXPECT_FALSE(circuit_breaker_enabled_by_val(0, CBRK_BUZZER_KEY));
	EXPECT_FALSE(circuit_breaker_enabled_by_val(CBRK_BUZZER_KEY, CBRK_IO_SAFETY_KEY));
	EXPECT_FALSE(circuit_breaker_enabled_by_val(-1, CBRK_USB_CHK_KEY));
}

TEST(CircuitBreakerVal, PublishedKeysDistinct)
{
	// Spot-check keys stay unique so a wrong match cannot silent-enable another breakers
	EXPECT_NE(CBRK_BUZZER_KEY, CBRK_SUPPLY_CHK_KEY);
	EXPECT_NE(CBRK_IO_SAFETY_KEY, CBRK_FLIGHTTERM_KEY);
	EXPECT_NE(CBRK_USB_CHK_KEY, CBRK_VTOLARMING_KEY);
}
