// SPDX-License-Identifier: BSD-3-Clause

#include <lib/dragonfly_turtle/ArmingInterlock.hpp>
dragonfly_trtl::Interlock *other_translation_unit()
{
	return &dragonfly_trtl::interlock();
}
