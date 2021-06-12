var target_bank_deg = 0;
var target_speed_kt = 50;
var target_alt_ft = 500;

var time = 0;
var last_time = 0.0;
var converge_time = 10000000.0; # a really big number of seconds we'd
				# never make it to in a real run of
				# the sim
var timer1 = 0.0;

var update_state = func( dt ) {
    if ( dt < 0.000001 ) {
        return;
    }

    var state = getprop("/flight-test/state");

    var bank_deg = getprop("/orientation/roll-deg");
    var speed_kt = getprop("/velocities/airspeed-kt");
    var alt_ft = getprop("/position/altitude-ft");

    if ( state == "init" ) {
	# turn on wing leveler
	setprop("/autopilot/locks/heading", "bank-hold");
	setprop("/autopilot/settings/target-bank-deg", target_bank_deg);
	# turn on altitude hold
	setprop("/autopilot/locks/altitude", "altitude-hold");
	# var target_alt_ft = int((alt_ft + 550) / 100) * 100;
	setprop("/autopilot/settings/target-altitude-ft", target_alt_ft );
	# turn on auto-throttle
	setprop("/autopilot/locks/speed", "speed-with-throttle");
	setprop("/autopilot/settings/target-speed-kt", target_speed_kt);
	setprop("/flight-test/state", "converging");
    } elsif ( state == "converging" ) {
	var bank_diff = abs( bank_deg - target_bank_deg );
	var speed_diff = abs( speed_kt - target_speed_kt );
	var alt_diff = abs( alt_ft - target_alt_ft );
	#printf("bank=%.1f spd=%.1f alt=%.1f t=%.1f ct=%.1f\n", bank_diff,
	#       speed_diff, alt_diff, time, converge_time);
	if ( (bank_diff < 0.1) and (speed_diff < 0.5) and (alt_diff < 5) and (time >= 20.0) )
	{
	    if ( converge_time > time ) {
		converge_time = time;
	    }
	} else {
	    converge_time = 10000000.0;
	}
	if ( time - converge_time > 10 ) {
	    setprop("/flight-test/state", "doublet1");
	    setprop("/autopilot/locks/heading", "");
	    setprop("/autopilot/locks/altitude", "");
	    setprop("/autopilot/locks/speed", "");
	    setprop("/controls/flight/aileron", "0.5");
	    timer1 = time;
	}
    } elsif ( state == "doublet1" ) {
	if ( time - timer1 > 1 ) {
	    setprop("/flight-test/state", "doublet2");
	    setprop("/controls/flight/aileron", "-0.5");
	    timer1 = time;
  	}
    } elsif ( state == "doublet2" ) {
	if ( time - timer1 > 2 ) {
	    setprop("/flight-test/state", "done");
	    setprop("/autopilot/locks/heading", "bank-hold");
	    #setprop("/autopilot/locks/altitude", "altitude-hold");
	    #setprop("/autopilot/locks/speed", "speed-with-throttle");
	}
    }
}

var main_loop = func {
    time = getprop("/sim/time/elapsed-sec");
    var dt = time - last_time;
    last_time = time;

    if ( getprop("/flight-test/master-switch") == 1 ) {
        update_state( dt );
    }

    settimer(main_loop, 0);
}

setlistener("/sim/signals/fdm-initialized",
            func {
                props.globals.initNode("/flight-test/master-switch", 1,
				       "BOOL", 1);
                setprop("/flight-test/state", "init");
                main_loop();
            });
