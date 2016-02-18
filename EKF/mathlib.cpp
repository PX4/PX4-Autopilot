#include "mathlib.h"

float math::constrain(float &val, float min, float max)
{
	return (val < min) ? min : ((val > max) ? max : val);
}
float math::radians(float degrees)
{
	return (degrees / 180.0f) * M_PI_F;
}
float math::degrees(float radians)
{
	return (radians * 180.0f) / M_PI_F;
}