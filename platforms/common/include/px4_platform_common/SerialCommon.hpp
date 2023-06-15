
#pragma once

namespace device
{
namespace SerialConfig
{


// ByteSize: number of data bits
enum class ByteSize {
	FiveBits  = 5,
	SixBits   = 6,
	SevenBits = 7,
	EightBits = 8,
};

// Parity: enable parity checking
enum class Parity {
	None = 0,
	Odd  = 1,
	Even = 2,
};

// StopBits: number of stop bits
enum class StopBits {
	One = 1,
	Two = 2
};

// FlowControl: enable flow control
enum class FlowControl {
	Disabled = 0,
	Enabled  = 1,
};

} // namespace Serial
} // namespace device
