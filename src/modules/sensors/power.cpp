//
// Created by timothy on 23/07/19.
//

#include "power.h"

Power::Power(sensors::Parameters *parameters) :  _parameters(parameters) {}

void Power::update(px4_adc_msg_t buf_adc[PX4_MAX_ADC_CHANNELS], int nchannels)
{

#if BOARD_NUMBER_BRICKS > 0
	/* For legacy support we publish the battery_status for the Battery that is
	 * associated with the Brick that is the selected source for VDD_5V_IN
	 * Selection is done in HW ala a LTC4417 or similar, or may be hard coded
	 * Like in the FMUv4
	 */


	/* Per Brick readings with default unread channels at 0 */
	int32_t bat_current_cnt[BOARD_NUMBER_BRICKS];
	int32_t bat_voltage_cnt[BOARD_NUMBER_BRICKS];

	/* Based on the valid_chan, used to indicate the selected the lowest index
	 * (highest priority) supply that is the source for the VDD_5V_IN
	 * When < 0 none selected
	 */

	int selected_source = -1;


	/* Read add channels we got */
	for (unsigned i = 0; i < nchannels / sizeof(buf_adc[0]); i++) {
		for (int b = 0; b < BOARD_NUMBER_BRICKS; b++) {
			/* look for specific channels and process the raw voltage to measurement data */
			if (_analogBatteries[b]->vChannel == buf_adc[i].am_channel) {
				/* Voltage in volts */
				bat_voltage_cnt[b] = buf_adc[i].am_data;

			} else if (_analogBatteries[b]->iChannel == buf_adc[i].am_channel) {
				bat_current_cnt[b] = buf_adc[i].am_data;
			}
		}
	}


	for (int b = 0; b < BOARD_NUMBER_BRICKS; b++) {

		//PX4_INFO("Battery %d. V: %ud, I: %ud", b, bat_voltage_cnt[b], bat_current_cnt[b]);

		// TODO: Fill in the rest of these params
		_analogBatteries[b]->updateBatteryStatus(bat_voltage_cnt[b], bat_current_cnt[b], hrt_absolute_time(),
				_analogBatteries[b]->channelValid, selected_source == b, b, 0.0f, false);
	}

#endif /* BOARD_NUMBER_BRICKS > 0 */

}