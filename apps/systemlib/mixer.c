/**
 * @file	Generic control value mixing library.
 *
 * This library implements a generic mixer function that can be used
 * by any driver or subsytem that wants to combine several control signals
 * into a single output.
 *
 * See mixer.h for more details.
 */

#include "mixer.h"

static int
scale_check(struct MixScaler *scale)
{
 	if (scale->offset > 1.0f)
 		return -1;
 	if (scale->offset > 1.0f)
 		return -1;
 	if (scale->lower_limit > scale->upper_limit)
 		return -1;
 	if (scale->lower_limit < -1.0f)
 		return -1;
 	if (scale->upper_limit > 1.0f)
 		return -1;
 	return 0;
}

int
mixer_check(struct MixMixer *mixer, unsigned control_count)
{
	if (mixer->control_count < 1)
		return -1;
	if (mixer->control_count > control_count)
		return -1;
	if (!scale_check(&mixer->output_scaler))
		return -1;

	for (unsigned i = 0; i < mixer->control_count; i++) {
		if (mixer->control_scaler[i].control >= control_count)
			return -1;
		if (!scale_check(&mixer->control_scaler[i]))
			return -1;
	}
	return 0;
}

static float
scale(struct MixScaler *scaler, float input)
{
	float output;

	if (input < 0.0f) {
		output = (input * scaler->negative_scale) + scaler->offset;
	} else {
		output = (input * scaler->positive_scale) + scaler->offset;
	}
	if (output > scaler->upper_limit) {
		output = scaler->upper_limit;
	} else if (output < scaler->lower_limit) {
		output = scaler->lower_limit;
	}

	return output;
}

float
mixer_mix(struct MixMixer *mixer, float *controls)
{
	struct MixScaler *scaler;
	float sum = 0.0f;

	for (unsigned i = 0; i < mixer->control_count; i++) {
		scaler = &mixer->control_scaler[i];
		sum += scale(scaler, controls[scaler->control]);
	}

	return scale(&mixer->output_scaler, sum);
}
