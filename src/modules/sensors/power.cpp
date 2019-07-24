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
#if !defined(BOARD_NUMBER_DIGITAL_BRICKS)
	/* The ADC channels that  are associated with each brick, in power controller
	 * priority order highest to lowest, as defined by the board config.
	 */
	int   bat_voltage_v_chan[BOARD_NUMBER_BRICKS] = BOARD_BATT_V_LIST;
	int   bat_voltage_i_chan[BOARD_NUMBER_BRICKS] = BOARD_BATT_I_LIST;

	if (_parameters->battery_adc_channel >= 0) {  // overwrite default
		bat_voltage_v_chan[0] = _parameters->battery_adc_channel;
	}

#endif

	/* The valid signals (HW dependent) are associated with each brick */
	bool  valid_chan[BOARD_NUMBER_BRICKS] = BOARD_BRICK_VALID_LIST;

	/* Per Brick readings with default unread channels at 0 */
	int32_t bat_current_cnt[BOARD_NUMBER_BRICKS];
	int32_t bat_voltage_cnt[BOARD_NUMBER_BRICKS];

	/* Based on the valid_chan, used to indicate the selected the lowest index
	 * (highest priority) supply that is the source for the VDD_5V_IN
	 * When < 0 none selected
	 */

	int selected_source = -1;

#endif /* BOARD_NUMBER_BRICKS > 0 */

	/* Read add channels we got */
	for (unsigned i = 0; i < nchannels / sizeof(buf_adc[0]); i++) {
		{

#if BOARD_NUMBER_BRICKS > 0

			for (int b = 0; b < BOARD_NUMBER_BRICKS; b++) {

				/* Once we have subscriptions, Do this once for the lowest (highest priority
				 * supply on power controller) that is valid.
				 */
				if (_battery_pub[b] != nullptr && selected_source < 0 && valid_chan[b]) {
					/* Indicate the lowest brick (highest priority supply on power controller)
					 * that is valid as the one that is the selected source for the
					 * VDD_5V_IN
					 */
					selected_source = b;

#  if BOARD_NUMBER_BRICKS > 1

					/* Move the selected_source to instance 0 */
					if (_battery_pub_intance0ndx != selected_source) {

						orb_advert_t tmp_h = _battery_pub[_battery_pub_intance0ndx];
						_battery_pub[_battery_pub_intance0ndx] = _battery_pub[selected_source];
						_battery_pub[selected_source] = tmp_h;
						_battery_pub_intance0ndx = selected_source;
					}

#  endif /* BOARD_NUMBER_BRICKS > 1 */
				}

#  if  !defined(BOARD_NUMBER_DIGITAL_BRICKS)

				// todo:per brick scaling
				/* look for specific channels and process the raw voltage to measurement data */
				if (bat_voltage_v_chan[b] == buf_adc[i].am_channel) {
					/* Voltage in volts */
					bat_voltage_cnt[b] = buf_adc[i].am_data;

				} else if (bat_voltage_i_chan[b] == buf_adc[i].am_channel) {
					bat_current_cnt[b] = buf_adc[i].am_data;
				}

#  endif /* !defined(BOARD_NUMBER_DIGITAL_BRICKS) */
			}

#endif /* BOARD_NUMBER_BRICKS > 0 */
		}
	}

#if BOARD_NUMBER_BRICKS > 0

	if (_parameters->battery_source == 0) {

		for (int b = 0; b < BOARD_NUMBER_BRICKS; b++) {

			// TODO: Fill in the rest of this status
			battery_status_s battery_status{};
			_battery[b].updateBatteryStatus(bat_voltage_cnt[b], bat_current_cnt[b], hrt_absolute_time(),
							valid_chan[b], selected_source == b, b, 0.0f,
							false, &battery_status);
			int instance;
			orb_publish_auto(ORB_ID(battery_status), &_battery_pub[b], &battery_status, &instance, ORB_PRIO_DEFAULT);
		}
	}

#endif /* BOARD_NUMBER_BRICKS > 0 */

}