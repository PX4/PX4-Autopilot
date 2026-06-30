# Parameter description pruning — Tier-2 follow-up

Follow-up to the parameter `short`/`long` description trim. These parameters were **left unchanged** in that pass because trimming them further would risk losing substance — each needs a maintainer's judgement. A few also flag likely pre-existing errors in the current descriptions (noted inline).

Total candidates: 82


## `src/drivers/actuators/voxl_esc/voxl_esc_params.yaml`

### VOXL_ESC_CONFIG — src/drivers/actuators/voxl_esc/voxl_esc_params.yaml
- current long length: 54 chars
- why further pruning is possible: "Selects what type of UART ESC, if any, is being used." is essentially a restatement; the values dict (Disabled / VOXL ESC) already covers the options
- aggressive proposal: delete long entirely
- substance at risk: "if any" signals that the ESC is optional/disabled by default, but default:0 already communicates that; deletion appears safe but reviewer should confirm no QGC rendering depends on the long being present


## `src/drivers/dshot/module.yaml`

### DSHOT_ESC_TYPE — src/drivers/dshot/module.yaml
- current long length: 21 chars
- why further pruning is possible: long "The ESC firmware type" nearly restates short "ESC Type"; the only addition is "firmware"
- aggressive proposal: drop long entirely
- substance at risk: "firmware" clarifies the type refers to software/protocol classification, not hardware make; may matter when multiple ESC hardware revisions share one protocol


## `src/drivers/ins/microstrain/module.yaml`

### MS_INT_HEAD_EN — src/drivers/ins/microstrain/module.yaml
- current long length: 82 chars
- why further pruning is possible: Could drop the hardware model name (CV7-GNSS/INS) and rely on MicroStrain docs.
- aggressive proposal: `On dual-antenna devices, uses dual-antenna heading as aiding.`
- substance at risk: The CV7-GNSS/INS model specificity tells users which device variant supports this feature.


## `src/drivers/ins/vectornav/module.yaml`

### VN_IMU_RATE — src/drivers/ins/vectornav/module.yaml
- current long length: 80 chars
- why further pruning is possible: The values: table already shows which divisor yields which Hz (1→800Hz, 2→400Hz etc.), so the divisor explanation is somewhat redundant.
- aggressive proposal: Drop long entirely.
- substance at risk: Without the long, users selecting an integer value won't understand why 1 yields 800 Hz (the base-rate/divisor relationship is implicit).


## `src/drivers/pca9685_pwm_out/module.yaml`

### PCA9685_PWM_FREQ — src/drivers/pca9685_pwm_out/module.yaml
- current long length: ~295 chars (after B12 trim)
- why further pruning is possible: "Should match update rate in most cases" is a general best-practice note; the 400 Hz hard cap and duty-cycle caveat are the only truly hard constraints.
- aggressive proposal: "Independent of update rate (PCA9685 outputs autonomously). Higher frequency = more accurate pulse width but may be unsupported by some ESCs/servos. Max 400 Hz for standard 1000-2000us output; >400 Hz duty-cycle mode only."
- substance at risk: Loses the "should match update rate" guidance that helps users avoid misconfiguration in the common case; the two parameters are independent but misconfiguring them causes latency/quality issues.


## `src/drivers/rpm/pcf8583/parameters.yaml`

### PCF8583_POOL — src/drivers/rpm/pcf8583/parameters.yaml
- current long length: 40 chars
- why further pruning is possible: long "Determines how often the sensor is read out." mostly restates short "PCF8583 rotorfreq (i2c) poll interval"
- aggressive proposal: drop long entirely
- substance at risk: negligible; short + unit (us) + default fully describe the parameter

### SENS_EN_PCF8583 — src/drivers/rpm/pcf8583/parameters.yaml
- current long length: 30 chars
- why further pruning is possible: long "Run PCF8583 driver automatically" nearly restates short "PCF8583 enable driver"; "automatically" is the only addition
- aggressive proposal: drop long entirely
- substance at risk: "automatically" clarifies the driver starts on boot without user action; could drop since it's implied by an enable parameter


## `src/drivers/uavcan/uavcan_params.yaml`

### UAVCAN_SUB_ASPD/BARO/GPS/etc — src/drivers/uavcan/uavcan_params.yaml
- current long length: ~50–100 chars each (message type list)
- why further pruning is possible: the DroneCAN message type strings (e.g. `uavcan::equipment::air_data::IndicatedAirspeed`) are protocol-internal identifiers; engineers can look them up by module name; removing them would save flash metadata
- aggressive proposal: remove all `long:` from subscription boolean params entirely
- substance at risk: direct cross-reference to the exact DroneCAN messages subscribed; useful for bus debugging without source access


## `src/drivers/vtx/module.yaml`

### VTX_BAND — src/drivers/vtx/module.yaml
- current long length: 18 chars
- why further pruning is possible: long "VTX table band 1-24" just restates that the enum covers bands 1-24, which is already visible in the 24 values
- aggressive proposal: drop long entirely
- substance at risk: "table" indicates these come from a VTX frequency table (not arbitrary RF bands), which may matter for advanced users

### VTX_CHANNEL — src/drivers/vtx/module.yaml
- current long length: 22 chars
- why further pruning is possible: long "VTX table channel 1-16" identical rationale to VTX_BAND
- aggressive proposal: drop long entirely
- substance at risk: same as VTX_BAND ("table" qualifier)

### VTX_POWER — src/drivers/vtx/module.yaml
- current long length: 30 chars
- why further pruning is possible: long "VTX transmission power level 1-16" restates short "VTX power level"; only addition is "transmission" and "1-16"
- aggressive proposal: drop long entirely
- substance at risk: "transmission" adds slight clarity that this is TX power (not receive sensitivity); minor


## `src/lib/adsb/parameters.yaml`

### ADSB_EMERGC — src/lib/adsb/parameters.yaml
- current long length: 31 chars
- why further pruning is possible: long "Sets the vehicle emergency state" restates short "ADSB-Out Emergency State"
- aggressive proposal: drop long entirely
- substance at risk: none; short + enum values fully describe the parameter

### ADSB_EMIT_TYPE — src/lib/adsb/parameters.yaml
- current long length: 38 chars
- why further pruning is possible: long "Configure the emitter type of the vehicle." restates short "ADSB-Out Vehicle Emitter Type"
- aggressive proposal: drop long entirely
- substance at risk: none; short + enum values fully describe the parameter

### ADSB_ICAO_ID — src/lib/adsb/parameters.yaml
- current long length: 34 chars
- why further pruning is possible: long "Defines the ICAO ID of the vehicle" restates short "ADSB-Out ICAO configuration"
- aggressive proposal: drop long entirely
- substance at risk: "vehicle" is explicit, but that's already implied by ADSB-Out context; negligible loss

### ADSB_IDENT — src/lib/adsb/parameters.yaml
- current long length: 40 chars
- why further pruning is possible: long "Enable Identification of Position feature" adds "Position" over short "ADSB-Out Ident Configuration"; borderline
- aggressive proposal: drop long; or "Activates the Ident of Position feature."
- substance at risk: "Position" is the official ICAO term for this feature; removing it may lose disambiguation from other Ident features

### ADSB_LIST_MAX — src/lib/adsb/parameters.yaml
- current long length: 32 chars
- why further pruning is possible: long "Change number of targets to track" essentially restates short "ADSB-In Vehicle List Size"
- aggressive proposal: drop long entirely
- substance at risk: negligible; short + type (int32) + min/max fully describe the parameter


## `src/lib/battery/module.yaml`

### BAT${i}_I_OVERWRITE — src/lib/battery/module.yaml
- current long length: 116 chars
- why further pruning is possible: could collapse to "Constant unarmed current override (A). <=0: disabled."
- substance at risk: the "Armed state always uses measured current" note clarifies the param only affects unarmed state, which isn't obvious from the short; and the ESC inaccuracy rationale helps calibrators decide when to use it

### BAT${i}_SOURCE — src/lib/battery/module.yaml
- current long length: 172 chars
- why further pruning is possible: the values labels (Power Module / Analog, External, ESCs) are reasonably self-explanatory; most of the long rephrases them
- aggressive proposal: drop to just "ESCs requires ESC telemetry with voltage and current."
- substance at risk: the note that Power Module/Analog covers both ADC and I2C (INA226) monitors; "External" covers MAVLink/CAN/external driver — meaningful distinctions for integrators


## `src/lib/circuit_breaker/circuit_breaker_params.yaml`

### CBRK_BUZZER — src/lib/circuit_breaker/circuit_breaker_params.yaml
- current long length: 165 chars
- why further pruning is possible: Uses verbose "Setting this parameter to N will …" pattern twice; can be rewritten as a two-value key list.
- aggressive proposal: "782097: disable all buzzer audio. 782090: disable startup tune only."
- substance at risk: Consistent pattern with other CBRKs; deviating could confuse users. Also, "while keeping all others enabled" is an implied detail worth preserving (captured in "startup tune only").

### CBRK_FLIGHTTERM — src/lib/circuit_breaker/circuit_breaker_params.yaml
- current long length: 253 chars
- why further pruning is possible: "Setting this parameter to 121212 will disable…" preamble could be shortened; the list of unaffected safeties is a long clause.
- aggressive proposal: "121212: disable flight termination triggered by FailureDetector or FMU loss. Does not affect RC loss, data link loss, geofence, or takeoff failure detection."
- substance at risk: The list of unaffected items is safety-critical; any accidental omission would be harmful. Needs careful human review.


## `src/lib/fw_performance_model/performance_model_params.yaml`

### FW_AIRSPD_MAX — src/lib/fw_performance_model/performance_model_params.yaml
- current long length: 71 chars
- why further pruning is possible: long "The maximal airspeed (calibrated airspeed) the user is able to command." almost entirely restates short "Maximum Airspeed (CAS)"; only "the user is able to command" adds any substance
- aggressive proposal: drop long entirely (short is self-explanatory), or "User-commanded CAS limit."
- substance at risk: "the user is able to command" distinguishes this as a pilot limit rather than a hardware/envelope limit; removing the long loses that nuance


## `src/lib/mixer_module/motor_params.yaml`

### THR_MDL_FAC — src/lib/mixer_module/motor_params.yaml
- current long length: ~195 chars (excluding the code fence lines)
- why further pruning is possible: the surrounding prose explanation could be cut to just the formula with a brief label; "where rel_thrust and rel_signal are normalized 0-1" might be inferred from the parameter names
- aggressive proposal: |-\n  `rel_thrust = factor * rel_signal^2 + (1-factor) * rel_signal`\n  (rel_thrust, rel_signal in [0,1])
- substance at risk: removing the prose explanation of the nonlinear model context (PWM-to-thrust) could make it unclear what the formula represents without reading the short again


## `src/lib/rover_control/rovercontrol_params.yaml`

### RO_YAW_RATE_CORR — src/lib/rover_control/rovercontrol_params.yaml
- current long length: 215 chars
- why further pruning is possible: application note about skid-steered/misaligned-wheel rovers is helpful but non-critical to understanding the parameter function
- aggressive proposal: "FF mapping correction for yaw rate controller. Increase (>1) if measured rate < setpoint, decrease (<1) otherwise."
- substance at risk: loses targeted guidance for the most common use case where this param needs tuning; users with skid-steer rovers may not know to look here


## `src/lib/systemlib/system_params.yaml`

### SYS_BL_UPDATE — src/lib/systemlib/system_params.yaml
- current long length: ~290 chars
- why further pruning is possible: the 5-bullet instruction list could be condensed to 2-3 sentences; individual bullets (e.g. "Enable this parameter", "Reboot the board") are self-evident in context
- aggressive proposal: `WARNING: do not cut power; recovery requires JTAG if interrupted. Insert SD, enable, reboot, wait ≥2 min; if board does not return check bootlog.txt on SD.`
- substance at risk: losing distinct step ordering or the specific "2 minutes" timeframe could confuse a user attempting recovery in the field


## `src/modules/airspeed_selector/airspeed_selector_params.yaml`

### ASPD_FP_T_WINDOW — src/modules/airspeed_selector/airspeed_selector_params.yaml
- current long length: ~185 chars (after B12 trim)
- why further pruning is possible: The trigger condition sentence is long; "within this window" is implied by the parameter name.
- aggressive proposal: "Triggers when airspeed drops while throttle increases and vehicle pitches down. Catches degrading blockages (e.g. icing). Requires accurate FW_THR_TRIM."
- substance at risk: Drops "change within this window" qualifier; could be read as an instantaneous check rather than a windowed one, obscuring the relationship between the parameter value and detection sensitivity.

### ASPD_FS_INNOV — src/modules/airspeed_selector/airspeed_selector_params.yaml
- current long length: ~204 chars (after B12 trim)
- why further pruning is possible: The clause "Large values indicate groundspeed-windspeed vs measured inconsistency" is an explanation of what "innovation" means, not a behavioral fact. Domain-familiar users can infer it.
- aggressive proposal: "Min innovation to trigger failsafe. Larger = less sensitive. Detection time scales with exceedance size (see ASPD_FS_INTEG)."
- substance at risk: Removes the only prose explanation of what the innovation signal represents; users unfamiliar with state-estimation terminology may not connect "innovation" to the groundspeed-windspeed-vs-measured discrepancy.


## `src/modules/commander/HealthAndArmingChecks/esc_check_params.yaml`

### MOTFAIL_C2T — src/modules/commander/HealthAndArmingChecks/esc_check_params.yaml
- current long length: 198 chars
- why further pruning is possible: Last sentence references "FD_ACT_LOW_OFF and FD_ACT_HIGH_OFF" but the actual offset param in the file is MOTFAIL_OFF. These appear to be stale/renamed param names. If confirmed stale, the last sentence can be rewritten to reference MOTFAIL_OFF and shortened.
- aggressive proposal: "Slope between expected steady-state current and normalized thrust. MOTFAIL_C2T is the expected current at 100% thrust; MOTFAIL_OFF offsets the threshold."
- substance at risk: If FD_ACT_LOW_OFF and FD_ACT_HIGH_OFF are real separate params elsewhere, removing them loses a cross-reference.


## `src/modules/commander/commander_params.yaml`

### COM_FLT_TIME_MAX — src/modules/commander/commander_params.yaml
- current long length: 201 chars
- why further pruning is possible: The "auto modes blocked" and "manual takeover" lines could be condensed into one clause.
- aggressive proposal: "RTL at limit; non-RTL/Land auto modes blocked, manual takeover OK. Warning every 1 min from 90%. -1 disables."
- substance at risk: Losing the explicit statement that RTL/Land are the only non-blocked auto modes.

### COM_GNSSLOSS_ACT — src/modules/commander/commander_params.yaml
- current long length: 201 chars
- why further pruning is possible: The trigger condition is complex with two sub-cases; the current trimmed text is already compact but could fold the two conditions more aggressively.
- aggressive proposal: "Triggers when GNSS count drops below SYS_HAS_NUM_GNSS (0=disabled) or two receivers disagree (SYS_HAS_NUM_GNSS=2 only). Values above Warning block arming."
- substance at risk: The "(otherwise warning only)" clause clarifies fallback behavior for the disagreement case when SYS_HAS_NUM_GNSS≠2; dropping it could mislead users.

### COM_POS_FS_EPH — src/modules/commander/commander_params.yaml
- current long length: 197 chars
- why further pruning is possible: The 2.5× hysteresis factor is specific but could be expressed more tersely.
- aggressive proposal: "EPH failsafe threshold (hover MC/VTOL). Hysteresis: 2.5× gap between validation and invalidation. Independent from EKF2_NOAID_TOUT. -1 disables."
- substance at risk: "Triggers failsafe when EPH exceeds this threshold" restatement could go, but removing it loses the direction-of-trigger clarity.

### COM_THROW_SPEED — src/modules/commander/commander_params.yaml
- current long length: 110 chars
- why further pruning is possible: The original mentioned "freefall detection" as part of the mechanism — the trimmed text omitted this detail.
- aggressive proposal: (current text is already the aggressive version)
- substance at risk: Original said motors arm "before detecting the freefall" — the freefall detection step is part of the throw-start sequence and may matter for tuning/debugging. Currently dropped.

### COM_VEL_FS_EVH — src/modules/commander/commander_params.yaml
- current long length: 152 chars
- why further pruning is possible: Same 2.5× hysteresis pattern; could be condensed further.
- aggressive proposal: "EVH failsafe threshold. Default suits MC; increase for FW. Hysteresis: 2.5× gap."
- substance at risk: "Triggers failsafe when EVH exceeds this threshold" is near-restate of short but confirms direction; aggressive removal risks ambiguity.


## `src/modules/control_allocator/module.yaml`

### CA_HELI_RPM_P — src/modules/control_allocator/module.yaml
- current long length: ~100 chars (after trim)
- why further pruning is possible: First line ("Normalized output added per (rpm_error / 1000).") already conveys the ratio; the second line repeats the same math in expanded form.
- aggressive proposal: Keep formula only, drop the prose first line: "motor_command = throttle_curve + CA_HELI_RPM_P * (rpm_setpoint - rpm_measurement) / 1000"
- substance at risk: The prose line is arguably clearer for non-engineers; dropping it trades readability for brevity.

### CA_HELI_YAW_CP_S / CA_HELI_YAW_TH_S — src/modules/control_allocator/module.yaml
- current long length: ~130 chars each (after trim)
- why further pruning is possible: Both end with the tail_output formula. Since CA_HELI_YAW_CP_O also carries that formula (with the same structure), the repetition in the other two params could be removed.
- aggressive proposal: Drop the formula line from CA_HELI_YAW_CP_S and CA_HELI_YAW_TH_S, keeping only the prose description.
- substance at risk: The formula is the precise mathematical definition for each param individually. Readers using only the per-param help text (e.g. QGC tooltip) would lose the math. Safe only if the full formula in CA_HELI_YAW_CP_O is considered sufficient context.


## `src/modules/ekf2/module.yaml`

### EKF2_HGT_REF — src/modules/ekf2/module.yaml
- current long length: ~220 chars (4-line block scalar with blank-line paragraph break after trim)
- why further pruning is possible: The GPS altitude bias-initialization note is technically important but could be one compact clause.
- aggressive proposal: "Height converges to this reference. Range/Vision require flat ground (NED origin moves with terrain). GPS selected + EKF2_GPS_CTRL != 0: GPS altitude still initializes other height biases regardless of the altitude bit."
- substance at risk: None identified beyond style; but changing to a single paragraph loses the structural separation between the general rule and the GPS-specific exception, which could be confusing.


## `src/modules/ekf2/params_drag.yaml`

### EKF2_MCOEF — src/modules/ekf2/params_drag.yaml
- current long length: ~310 chars (4-line block scalar after trim)
- why further pruning is possible: The physical mechanism (air velocity through rotor disc → momentum change → drag) is a second-principles explanation that experienced engineers may not need. "Ducted rotors" note is brief but could fold into a parenthetical.
- aggressive proposal: "Propeller momentum drag coefficient (scales with v, not v²). Higher for ducted rotors than open propellers. For bluff-body drag (v²) see EKF2_BCOEF_X/Y. 0 disables for both axes."
- substance at risk: The "proportional to blade side-on area" detail helps engineers estimate the coefficient from geometry; removing it loses a calibration cue.


## `src/modules/ekf2/params_magnetometer.yaml`

### EKF2_MAG_TYPE — src/modules/ekf2/params_magnetometer.yaml
- current long length: ~320 chars (4-line block scalar after trim)
- why further pruning is possible: The "None" mode GNSS velocity alignment note could be moved to a bit label on the None enum entry, and the Automatic/Init lines partly restate what the enum labels already convey.
- aggressive proposal: "3-axis fusion can learn body-fixed hard iron errors but requires a stable earth field. None: yaw can be aligned post-takeoff via horizontal movement and GNSS velocity."
- substance at risk: Dropping "Automatic: heading on-ground, 3-axis in-flight" and "Init: mag used only to initialize heading" from the long would leave those distinctions only in the enum labels (which are single words: "Automatic", "Init"). Users relying on the long for context would lose that detail unless enum labels are expanded.

### EKF2_SYNT_MAG_Z — src/modules/ekf2/params_magnetometer.yaml
- current long length: ~235 chars (3-line block scalar after trim)
- why further pruning is possible: The two-mode distinction (heading vs 3D fusion behaves differently) is important but verbose. Could be one sentence.
- aggressive proposal: "Replaces measured Z with a synthetic value in heading fusion (requires known global position); ignores measured Z in 3D fusion."
- substance at risk: Loses "for Z-axis magnetic interference" use-case context that helps users know when to enable this.


## `src/modules/ekf2/params_range_finder.yaml`

### EKF2_RNG_CTRL — src/modules/ekf2/params_range_finder.yaml
- current long length: ~335 chars (5-line block scalar after trim)
- why further pruning is possible: The WARNING sentence could be cut to just "WARNING: Less reliable than baro; prefer MPC_ALT_MODE for precise ground-relative height." and the conditional mode explanation could drop "for vertical takeoff/landing" since the EKF2_RNG_A_VMAX/HMAX refs already imply the use case.
- aggressive proposal: "WARNING: Less reliable; prefer MPC_ALT_MODE unless baro errors affect landing/takeoff. Fuses range as additional height. Conditional mode (value 1): below EKF2_RNG_A_VMAX and EKF2_RNG_A_HMAX only."
- substance at risk: Safety context (unexpected errors, rotor wash corrupts baro) might be important for operators choosing this mode; dropping it reduces the rationale for the caution.


## `src/modules/flight_mode_manager/tasks/AutoFollowTarget/follow_target_params.yaml`

### FLW_TGT_ALT_M — src/modules/flight_mode_manager/tasks/AutoFollowTarget/follow_target_params.yaml
- current long length: 142 chars
- why further pruning is possible: The terrain-crash warning for modes 0/1 could potentially be inferred from the value labels ("Maintain constant altitude" vs. the caveat in mode 2's label "GPS altitude bias usually makes this useless"). The FLW_TGT_HT cross-reference could arguably be dropped.
- aggressive proposal: `Modes 0/1 risk terrain collision if target ascends. Mode 2: use large FLW_TGT_HT.`
- substance at risk: Drops the "GPS altitude inaccuracies" explanation for why mode 2 needs a large FLW_TGT_HT, which is not implied by the value label.

### FLW_TGT_FA — src/modules/flight_mode_manager/tasks/AutoFollowTarget/follow_target_params.yaml
- current long length: 103 chars
- why further pruning is possible: The wrapping example "480→120" is illustrative but not strictly necessary since the min/max fields already bound the range.
- aggressive proposal: `0 = in front of target's motion; clockwise (right=90, left=-90). Out-of-range values are wrapped.`
- substance at risk: Drops the concrete example of wrapping (480→120), which helps users understand the wrap direction/magnitude.


## `src/modules/fw_att_control/fw_att_control_params.yaml`

### FW_MAN_YR_MAX — src/modules/fw_att_control/fw_att_control_params.yaml
- current long length: 85 chars
- why further pruning is possible: The phrase "attitude-controlled modes" is already implicit from context; the turn-coordination reference could be reduced.
- aggressive proposal: `Yaw stick contribution added on top of the controller's turn-coordination rate.`
- substance at risk: Dropping "in attitude-controlled modes" removes the qualifier that clarifies this does not apply in acro/manual modes.


## `src/modules/fw_lateral_longitudinal_control/fw_lat_long_params.yaml`

### FW_T_THR_LOW_HGT — src/modules/fw_lateral_longitudinal_control/fw_lat_long_params.yaml
- current long length: 183 chars
- why further pruning is possible: long contains two cross-param references (FW_LND_THRTC_SC, FW_T_ALT_TC) and a specific transition rate (1 sec/sec); the prose could be compacted with abbreviations
- aggressive proposal: "Below this height, TECS transitions altitude TC from FW_T_ALT_TC→FW_LND_THRTC_SC*FW_T_ALT_TC at 1s/s. -1 disables."
- substance at risk: "smoothly (1 sec / sec)" distinguishes this from an instantaneous step; the compound cross-reference FW_LND_THRTC_SC*FW_T_ALT_TC must survive exactly

### FW_WIND_ARSP_SC — src/modules/fw_lateral_longitudinal_control/fw_lat_long_params.yaml
- current long length: 228 chars
- why further pruning is possible: the two-sentence long can be collapsed into one; the multiplicative relationship could use notation
- aggressive proposal: "Factor × wind estimate = airspeed offset added to minimum airspeed setpoint; improves robustness in turbulence."
- substance at risk: "current absolute wind estimate" specifies it uses the real-time wind (not a fixed value) — "× wind estimate" must preserve that; "minimum airspeed setpoint limit" (not just setpoint) should survive


## `src/modules/fw_mode_manager/fw_mode_manager_params.yaml`

### FW_LND_ABORT — src/modules/fw_mode_manager/fw_mode_manager_params.yaml
- current long length: ~102 chars
- why further pruning is possible: The long now says "Last terrain estimate (last valid or land waypoint) is used as ground until abort. Bits ignored if FW_LND_USETER == 0." The TODO comment was dropped (developer note, not user substance). Could further condense.
- aggressive proposal: "Bits ignored if FW_LND_USETER == 0. Uses last valid estimate or land waypoint as ground reference until abort."
- substance at risk: Minimal — just word reorder. Flagged for awareness.

### FW_LND_NUDGE — src/modules/fw_mode_manager/fw_mode_manager_params.yaml
- current long length: ~123 chars
- why further pruning is possible: Long describes the yaw stick direction convention (stick right = land point right). The value labels now carry angle vs path distinction. The direction note could be cut.
- aggressive proposal: "Corrects map/GNSS landing offset with yaw stick, constrained to FW_LND_TD_OFF."
- substance at risk: The vehicle-relative direction hint ("stick right = land point right") is useful for pilots and not obvious; dropping it removes operational guidance.


## `src/modules/fw_rate_control/fw_rate_control_params.yaml`

### FW_ACRO_YAW_EN — src/modules/fw_rate_control/fw_rate_control_params.yaml
- current long length: 148 chars
- why further pruning is possible: last sentence "When disabled, pilot directly commands the yaw actuator" is nearly implied by the preceding clause; dropping it would save ~54 chars
- aggressive proposal: "Enables yaw rate controller in FW Acro; disabled by default because an active controller fights the plane's natural turn coordination."
- substance at risk: explicit clarification of what control authority the pilot retains when the parameter is 0; some users may not intuitively know it reverts to direct actuator command

### FW_RLL_TO_YAW_FF — src/modules/fw_rate_control/fw_rate_control_params.yaml
- current long length: 117 chars
- why further pruning is possible: second sentence "Applies yaw actuator to counteract this." restates the function already described by the short (feedforward) and first sentence
- aggressive proposal: "Counteracts adverse yaw: when the plane rolls, the nose yaws out of the turn."
- substance at risk: slightly loses the explicit mechanism (yaw actuator) but the short "Roll control to yaw control feedforward gain" and the param name already imply it


## `src/modules/gimbal/gimbal_params.yaml`

### MNT_TAU — src/modules/gimbal/gimbal_params.yaml
- current long length: 142 chars (trimmed version)
- why further pruning is possible: "tune for the specific servo's speed and response" is generic tuning advice that applies to all open-loop servo params; could be shortened to "tune per servo".
- aggressive proposal: `Alpha filter convergence rate (s) for open-loop AUX mount control. Use when no position feedback is available.`
- substance at risk: Drops the explicit tuning reminder; less important but provides context for users unfamiliar with alpha-filter tuning.


## `src/modules/landing_target_estimator/landing_target_estimator_params.yaml`

### LTEST_ACC_UNC / LTEST_MEAS_UNC — src/modules/landing_target_estimator/landing_target_estimator_params.yaml
- current long length: ~120 chars each (trimmed)
- why further pruning is possible: The "Higher = ..." guidance lines are informal calibration hints that could be dropped if the parameter names (Acceleration uncertainty, Measurement uncertainty) are self-explanatory to Kalman filter users.
- aggressive proposal (ACC_UNC): `Variance of acceleration measurement for landing target position prediction.`
- aggressive proposal (MEAS_UNC): `Variance of the landing target measurement from the driver.`
- substance at risk: Drops the only guidance on how changing these values affects tracking behavior vs. outlier rejection — important for users tuning the estimator who aren't familiar with the underlying filter.


## `src/modules/local_position_estimator/params.yaml`

### LPE_ACC_XY — src/modules/local_position_estimator/params.yaml
- current long length: 92 chars
- why further pruning is possible: "Data sheet noise density = 150ug/sqrt(Hz) = 0.0015 m/s^2/sqrt(Hz)" repeats the default value in textual form
- aggressive proposal: "Set larger than data sheet to account for tilt error."
- substance at risk: the specific data sheet value (150ug/sqrt(Hz) = 0.0015 m/s^2/sqrt(Hz)) is a calibration reference that explains why the default of 0.012 is larger than spec; removing it leaves users with no baseline to judge their setting

### LPE_T_MAX_GRADE — src/modules/local_position_estimator/params.yaml
- current long length: 90 chars (after trim)
- why further pruning is possible: the degree equivalents (100 = 45 deg, 0 = 0 deg) could be dropped since the unit field says %
- aggressive proposal: "Hilly/outdoor: 100; flat/indoor: 0. Used to calculate increased terrain random walk noise due to movement."
- substance at risk: the degree conversion (100% grade = 45 deg) is non-obvious and genuinely useful for users setting this in real terrain; dropping it removes that reference


## `src/modules/logger/module_params_crypto.yaml`

### SDLOG_KEY — src/modules/logger/module_params_crypto.yaml
- current long length: ~199 chars (after B12 trim)
- why further pruning is possible: The phrase "stored RSA2048-encrypted on SD alongside the logfile using SDLOG_EXCHANGE_KEY" largely duplicates the content of SDLOG_EXCH_KEY's description.
- aggressive proposal: "Keystore index for the log encryption key. For symmetric algorithms: generated at log start, volatile (log-duration only); exchange key set via SDLOG_EXCHANGE_KEY."
- substance at risk: The current wording makes explicit that the encrypted key is stored ON SD alongside the logfile, which is a security-relevant deployment detail not obvious from SDLOG_EXCH_KEY alone.


## `src/modules/mavlink/module.yaml`

### MAV_${i}_RATE — src/modules/mavlink/module.yaml
- current long length: ~115 chars
- why further pruning is possible: The 8N1 bandwidth explanation (`baudrate/10 = max`) could be dropped as advanced detail most users won't need.
- aggressive proposal: `"Max rate for all MAVLink streams; throttled proportionally if exceeded. 0: auto (baudrate/20)."`
- substance at risk: The 8N1/baudrate relationship is needed to configure high-rate telemetry links correctly.


## `src/modules/mc_hover_thrust_estimator/hover_thrust_estimator_params.yaml`

### HTE_HT_ERR_INIT — src/modules/mc_hover_thrust_estimator/hover_thrust_estimator_params.yaml
- current long length: 79 chars
- why further pruning is possible: Long reads "Sets the number of standard deviations used by the innovation consistency test." — identical to HTE_ACC_GATE's long, which is about a gate size. HTE_HT_ERR_INIT's short is "1-sigma initial hover thrust uncertainty"; the long describes an unrelated concept and appears to be a copy-paste bug. Could drop long entirely (short is self-sufficient).
- aggressive proposal: (drop long)
- substance at risk: If the long is intentional (i.e., this param genuinely also sets std-devs for a consistency test), dropping it loses that info. Needs domain review to confirm it's a bug vs. a dual-purpose param.


## `src/modules/mc_pos_control/multicopter_autonomous_params.yaml`

### MPC_XY_ERR_MAX — src/modules/mc_pos_control/multicopter_autonomous_params.yaml
- current long length: ~112 chars
- why further pruning is possible: "Trajectory integration slows linearly with position error; stops when error >= this. Adjust based on vehicle tracking capability." The "Adjust based on vehicle tracking capability" is slightly generic but is genuine tuning guidance.
- aggressive proposal: "Trajectory integration slows linearly with position error; stops when error >= this."
- substance at risk: The tuning guidance sentence, while generic, may help users know this is a tunable parameter and not a hard system limit.


## `src/modules/mc_pos_control/multicopter_position_mode_params.yaml`

### MPC_ACC_HOR_MAX — src/modules/mc_pos_control/multicopter_position_mode_params.yaml
- current long length: ~65 chars
- why further pruning is possible: The long references "MPC_POS_MODE\n1 just deceleration\n4 not used, use MPC_ACC_HOR instead". Mode 1 does not exist in the enum values (only 0 and 4), making this likely stale documentation from a removed mode. The text is cryptic and could be rewritten.
- aggressive proposal: "In MPC_POS_MODE 0 (Direct velocity): deceleration only. In MPC_POS_MODE 4 (Acceleration based): not used — set MPC_ACC_HOR instead."
- substance at risk: "1 just deceleration" may intentionally reference a removed mode that still has behavioral meaning, or it may be a bug. Rewriting without confirming the code behavior risks losing a real constraint.


## `src/modules/mc_rate_control/mc_acro_params.yaml`

### MC_ACRO_SUPEXPO / MC_ACRO_SUPEXPOY — src/modules/mc_rate_control/mc_acro_params.yaml
- current long length: ~100 chars each (after trim)
- why further pruning is possible: the 0.7 example entry could be removed as mid-range guidance
- aggressive proposal: "Refines curve shape set by MC_ACRO_EXPO.\n0: pure expo\n0.95: very strong, near-maxima effect only"
- substance at risk: the 0.7 example ("reasonable enhancement for intuitive stick feel") provides a useful practical starting point; without it users only have the extremes


## `src/modules/mc_rate_control/mc_rate_control_params.yaml`

### MC_BAT_SCALE_EN — src/modules/mc_rate_control/mc_rate_control_params.yaml
- current long length: 199 chars
- why further pruning is possible: the concrete example ("e.g. 0.5 at both 100% and 60% charge") can be removed
- aggressive proposal: "Compensates for battery voltage drop to normalize performance across the operating range, keeping hover throttle constant with reduced max acceleration at lower charge."
- substance at risk: the numerical example (0.5 throttle at 100% and 60%) illustrates "constant hover throttle" concretely; removing it means users must infer the behavior without a reference point

### MC_ROLLRATE_K / MC_PITCHRATE_K / MC_YAWRATE_K — src/modules/mc_rate_control/mc_rate_control_params.yaml
- current long length: ~170 chars each (after trim)
- why further pruning is possible: the code-block formula occupies ~90 chars; the ideal/parallel distinction could be compressed further
- aggressive proposal: remove the code block entirely, keep only "Global gain scaling all PID terms. Set MC_*RATE_P=1 for ideal form; MC_*RATE_K=1 for parallel form."
- substance at risk: the formula is the clearest way to show exactly how K scales P, I, D; removing it loses the precise mathematical relationship


## `src/modules/navigator/mission_params.yaml`

### MIS_YAW_TMT — src/modules/navigator/mission_params.yaml
- current long length: ~213 chars
- why further pruning is possible: The interaction between ">0 skips heading check for normal waypoints" and the forced-heading timeout is the tricky part. The first sentence could be tighter.
- aggressive proposal: ">0: skip heading check for normal waypoints; wait this long for forced-heading waypoints (e.g. VTOL forward transition). For VTOLs with limited yaw authority. -1: disabled."
- substance at risk: The current phrasing separates the two behaviors (forced-heading timeout vs. normal waypoint skip) clearly; merging them risks ambiguity about which behavior applies when.


## `src/modules/navigator/navigator_params.yaml`

### NAV_MIN_LTR_ALT — src/modules/navigator/navigator_params.yaml
- current long length: ~175 chars
- why further pruning is possible: First sentence is still long; "e.g. via RC switch" could be dropped as an example.
- aggressive proposal: "Min altitude above Home in Loiter/Hold when entered without altitude. Does not apply to mission loiters or reposition setpoints. Negative: disabled."
- substance at risk: The "e.g. via RC switch" is a useful concrete example for users trying to understand when this applies vs. not.


## `src/modules/navigator/rtl_params.yaml`

### RTL_TYPE — src/modules/navigator/rtl_params.yaml
- current long length: 109 chars
- why further pruning is possible: The long is a meta-summary of what the values dict already describes in full detail. It could be dropped entirely.
- aggressive proposal: (drop long)
- substance at risk: The one-liner does provide a quick orientation ("home location, rally point, mission landing pattern, reverse mission") that the values cover individually but not as a compact overview.


## `src/modules/rover_differential/module.yaml`

### RD_TRANS_DRV_TRN — src/modules/rover_differential/module.yaml
- current long length: 195 chars (trimmed version)
- why further pruning is possible: The second sentence (waypoint smooth-stop behavior) is complex and could be moved to documentation; the mathematical relationship `< 180 - RD_TRANS_DRV_TRN` might be confusing in a parameter description.
- aggressive proposal: `Yaw error above which the state machine switches from driving to turning. Also used as the smooth-stop threshold at waypoints.`
- substance at risk: Drops the specific geometric condition (angle between line segments < 180° − this param) that determines when smooth-stop activates; users won't know the exact condition without it.


## `src/modules/sensors/module.yaml`

### CAL_MAG${i}_XCOMP / YCOMP / ZCOMP — src/modules/sensors/module.yaml
- current long length: ~115 chars each (after trim)
- why further pruning is possible: The per-axis mention (X/Y/Z) in the long is redundant with the short; the unit info is the only unique content.
- aggressive proposal: `Throttle or current compensation coeff (CAL_MAG_COMP_TYP). Throttle: [G]; current: [G/kA].`
- substance at risk: Removing "body-frame X/Y/Z mag" loses the body-frame context. The short says "X/Y/Z Axis throttle compensation" which implies body-frame, but verifying that both throttle *and* current compensation are controlled here is the key substance.


## `src/modules/sensors/sensor_params_mag.yaml`

### SENS_MAG_SIDES — src/modules/sensors/sensor_params_mag.yaml
- current long length: ~173 chars (after trim)
- why further pruning is possible: The bit-value list (tail_down=1, etc.) exists solely to explain why the enum values (34, 38, 63) have those numbers. A user who knows the formula doesn't need them.
- aggressive proposal: Drop the bit-value line entirely, leaving only: `Two-side calibration estimates offsets only (scale unchanged); initial six-side calibration recommended.`
- substance at risk: Users have no way to construct custom combinations or verify the pre-set values (34/38/63) without the bit table. The enum-only form gives no insight into what orientations "two side" or "three side" actually use.


## `src/modules/sensors/vehicle_angular_velocity/imu_gyro_parameters.yaml`

### IMU_DGYRO_CUTOFF — src/modules/sensors/vehicle_angular_velocity/imu_gyro_parameters.yaml
- current long length: ~181 chars
- why further pruning is possible: The rationale ("D-term is most noise-susceptible; filtering it allows a higher IMU_GYRO_CUTOFF, reducing latency and permitting higher P gains") is somewhat verbose but carries genuine tuning guidance.
- aggressive proposal: `Cutoff for the 2nd-order Butterworth D-term filter. Lower cutoff allows raising IMU_GYRO_CUTOFF for less latency and higher P gains. 0 disables.`
- substance at risk: Drops the explanation of *why* D-term is noise-susceptible and the causal chain (filter → raise IMU_GYRO_CUTOFF → reduced latency). Could confuse tuners unfamiliar with D-term filter interactions.


## `src/modules/sensors/vehicle_gps_position/module.yaml`

### SENS_GPS_PRIME — src/modules/sensors/vehicle_gps_position/module.yaml
- current long length: ~224 chars
- why further pruning is possible: The EKF-wait behavior sentence could merge with the timeout sentence.
- aggressive proposal: `Preferred GPS instance when blending is inactive; secondary used only if primary times out. Set to DroneCAN node ID to select a DroneCAN GPS. No effect when blending is active.`
- substance at risk: The current phrasing "EKF waits for primary even if secondary is available" is a distinct and important behavior note; collapsing it with the timeout clause could obscure that the EKF actively holds off the secondary even when it's ready.


## `src/modules/simulation/simulator_sih/sih_params.yaml`

### SIH_LOC_H0 — src/modules/simulation/simulator_sih/sih_params.yaml
- current long length: ~174 chars
- why further pruning is possible: FlightGear note is niche (specific to one visualizer). The consistency note is already in LAT0 and LON0 longs.
- aggressive proposal: "AMSL altitude at simulation start. Adjust so vehicle sits on ground at takeoff (FlightGear). Must be consistent with LAT0, LON0, MU_X/Y/Z."
- substance at risk: The FlightGear tip is only relevant to FG users but genuinely useful to them; dropping it loses actionable guidance.


## `src/modules/spacecraft/spacecraft_attitude_params.yaml`

### SC_YAW_WEIGHT — src/modules/spacecraft/spacecraft_attitude_params.yaml
- current long length: ~165 chars
- why further pruning is possible: The "why" sentence (yaw has less authority, non-critical for hover) is background context rather than operational fact.
- aggressive proposal: `"Fraction [0,1] deprioritizing yaw vs. roll/pitch in non-linear control. No impact on SC_YAW_P gain."`
- substance at risk: The rationale helps tuners understand when to change this value; dropping it leaves the guidance unclear.


## `src/modules/spacecraft/spacecraft_rate_params.yaml`

### SC_BAT_SCALE_EN — src/modules/spacecraft/spacecraft_rate_params.yaml
- current long length: ~150 chars
- why further pruning is possible: The example (`hover at 0.5 throttle at 100% stays at 0.5 at 60%`) is illustrative but not strictly necessary.
- aggressive proposal: `"Scales throttle to compensate for battery voltage drop, normalizing performance across SoC."`
- substance at risk: The concrete example helps users understand the normalization intent; without it the description is abstract.

### SC_ROLLRATE_K / SC_PITCHRATE_K / SC_YAWRATE_K — src/modules/spacecraft/spacecraft_rate_params.yaml
- current long length: ~175 chars each (after trimming)
- why further pruning is possible: The inline formula (`output = K * (P * error + I * error_integral + D * error_derivative)`) is verbose; the ideal/parallel distinction could be expressed as a one-liner without the full expression.
- aggressive proposal: `"Multiplies all PID terms. P=1: ideal form; K=1: parallel form."`
- substance at risk: The full formula is useful for users implementing custom gain structures; dropping it loses the exact operational definition.


## `src/modules/uxrce_dds_client/module.yaml`

### UXRCE_DDS_AG_IP — src/modules/uxrce_dds_client/module.yaml
- current long length: 135 chars
- why further pruning is possible: the two numeric examples (192.168.1.2 and 127.0.0.1) take up most of the length; a user can compute any IP via the same formula
- aggressive proposal: `Ethernet uXRCE-DDS only. Must be int32 (not decimal dot notation); e.g. 127.0.0.1 = 2130706433.`
- substance at risk: dropping one of the two examples (especially the negative-value example -1062731518) removes the only illustration that int32 can be negative for IPs in the 192.168.x.x range, which is non-obvious


## `src/modules/vision_target_estimator/vision_target_estimator_params.yaml`

### VTE_ACC_D_UNC — src/modules/vision_target_estimator/vision_target_estimator_params.yaml
- current long length: ~210 chars (after B03 trim)
- why further pruning is possible: The formula `1-sigma vel_uav change over t: sqrt(VTE_ACC_D_UNC * t)` and "Variance rate (not per-step)" could be dropped, leaving only the tuning direction.
- aggressive proposal: `"White-noise PSD (m^2/s^3) on UAV acceleration input. Higher values: tighter tracking, fewer outlier rejections."`
- substance at risk: The formula is the mathematical definition needed to physically interpret the parameter and set it from IMU noise specs. "Not per-step / independent of update rate" clarifies a common calibration mistake.

### VTE_ACC_T_UNC — src/modules/vision_target_estimator/vision_target_estimator_params.yaml
- current long length: ~290 chars (after B03 trim)
- why further pruning is possible: Formula + numeric examples (0.1 default → ~0.32 m/s^2 at 1s) could be removed.
- aggressive proposal: `"Process-noise PSD for target acceleration random walk (moving-target builds only). Unit: ((m/s^2)^2)/s. Higher: follows manoeuvres more aggressively, accepts more outliers."`
- substance at risk: The examples concretely illustrate what the parameter means physically; without them it is hard to set from first principles. The formula ties the value to measurable acceleration change rates.

### VTE_BIAS_UNC — src/modules/vision_target_estimator/vision_target_estimator_params.yaml
- current long length: ~330 chars (after B03 trim)
- why further pruning is possible: The tuning guidance sentence ("Tune to realistic drift of the absolute reference…") and the formula+examples could be shortened or removed.
- aggressive proposal: `"Process-noise PSD (m^2/s) for GPS/vision bias random walk. Higher: bias adapts faster; lower: stiffer."`
- substance at risk: The drift examples (few cm over tens of seconds for consumer GNSS, less for RTK) give a concrete calibration target that distinguishes consumer GPS from RTK use cases. Losing this could lead to mis-tuning.

### VTE_BIA_AVG_TOUT — src/modules/vision_target_estimator/vision_target_estimator_params.yaml
- current long length: ~295 chars (after B03 trim)
- why further pruning is possible: "GNSS measurements propagate to match the vision timestamp" is an implementation detail that could be dropped; the high-level behavior is sufficient for tuning.
- aggressive proposal: |-
    ```
    Max time averaging the initial GNSS bias (while GNSS+vision are jointly observable).
    If the GNSS sample expires before convergence, the current LPF bias is activated.
    0 disables averaging (immediate activation).
    ```
- substance at risk: The propagation detail ("GNSS measurements propagate to match the vision timestamp") explains a latency-compensation mechanism; omitting it could confuse developers debugging synchronization issues.

### VTE_POS_NIS_THRE / VTE_YAW_NIS_THRE — src/modules/vision_target_estimator/vision_target_estimator_params.yaml
- current long length: ~160 chars each (after B03 trim)
- why further pruning is possible: The full probability table (6 values) could be replaced with the most common reference point.
- aggressive proposal: `"Chi-squared 1-DOF NIS threshold; lower = more rejections. Default 3.84 corresponds to 5% false-rejection rate."`
- substance at risk: The table lets the user select non-default thresholds (e.g., 6.63 for 1%) without consulting external chi-squared tables. Removing the table forces a lookup to change the value meaningfully.

### VTE_YAW_ACC_UNC — src/modules/vision_target_estimator/vision_target_estimator_params.yaml
- current long length: ~280 chars (after B03 trim)
- why further pruning is possible: Formula + example numbers could be dropped.
- aggressive proposal: `"White-noise PSD (rad^2/s^3) on yaw acceleration (drives yaw-rate state). Higher: follows rapid heading changes; lower: aggressive smoothing."`
- substance at risk: Same as VTE_ACC_D_UNC/VTE_ACC_T_UNC — the numerical examples help calibrate the parameter against known heading-change rates.


## `src/modules/vtol_att_control/standard_params.yaml`

### VT_FWD_THRUST_EN — src/modules/vtol_att_control/standard_params.yaml
- current long length: 236 chars (trimmed version)
- why further pruning is possible: "Descend is treated as Landing" is a minor edge-case note that could be folded into the value labels (e.g. value 1 "Enabled (except LANDING/Descend)") or dropped if the values already enumerate Landing vs non-Landing variants.
- aggressive proposal: Drop the "Descend is treated as Landing" sentence; rely on enum values (1–6 already distinguish LANDING vs non-LANDING cases).
- substance at risk: Users may not know Descend maps to Landing without this note, leading to unexpected behavior when descending.


## `src/modules/vtol_att_control/vtol_att_control_params.yaml`

### VT_FW_QC_HMAX — src/modules/vtol_att_control/vtol_att_control_params.yaml
- current long length: 175 chars
- why further pruning is possible: The safety-rationale sentence ("At high altitude, quad-chuting risks battery depletion and crash") is explanatory prose about *why* the limit exists, which is arguably secondary to the *what*.
- aggressive proposal: `Max height above ground (or Home/local origin if ground unavailable) where quad-chute can trigger.`
- substance at risk: Drops the explicit safety warning about battery depletion at high altitude — users might not understand the consequence of setting this high without it.


## `src/modules/zenoh/module.yaml`

### ZENOH_ENABLE — src/modules/zenoh/module.yaml
- current long length: 35 chars (after trimming)
- why further pruning is possible: The long (`Starts the Zenoh-Pico Node driver.`) already duplicates most of the short (`Enable Zenoh`). Could be deleted entirely.
- aggressive proposal: (delete long)
- substance at risk: "Zenoh-Pico Node" is the precise name of the component; without it, users searching docs for that term won't know which param controls it.

