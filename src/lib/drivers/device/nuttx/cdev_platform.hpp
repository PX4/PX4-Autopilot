
#pragma once

#include <cinttypes>

#include <nuttx/arch.h>

#include <px4_micro_hal.h>

#define ATOMIC_ENTER irqstate_t flags = px4_enter_critical_section()
#define ATOMIC_LEAVE px4_leave_critical_section(flags)

namespace device
{

using px4_file_operations_t = struct file_operations;
using file_t = struct file;

} // namespace device
