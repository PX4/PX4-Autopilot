/**
 * Enable fixed-wing MPC avoidance prototype.
 *
 * When disabled the module republishes incoming setpoints unchanged.
 *
 * @value 0 Disabled
 * @value 1 Enabled
 * @min 0
 * @max 1
 * @reboot_required true
 * @group FW MPC Avoidance
 */
PARAM_DEFINE_INT32(FW_MPC_AVOID_EN, 0);

/**
 * Internal MPC model integration step [s].
 *
 * Sets the integration step used by the internal SIH-like model during rollouts.
 *
 * @unit s
 * @min 0.005
 * @max 0.05
 * @decimal 3
 * @group FW MPC Avoidance
 */
PARAM_DEFINE_FLOAT(FW_MPC_AVOID_DT, 0.02f);
