props.globals.initNode("/avior/settings/ap-enable", 0, "BOOL", 1);
props.globals.initNode("/avior/settings/turret-enable", 0, "BOOL", 1);

var avior_last_time = 0.0;

var last_pos = geo.aircraft_position();


var task_avior_launch = func() {
    printf("operator requested launch\n");

    # disable internal nasal autopilot system
    setprop("/uas/master-switch", 0);
    setprop("/uas/state", "");

    # enable external avior control
    setprop("/avior/settings/ap-enable", 1);
    setprop("/avior/settings/turret-enable", 0);

    # setup initial state of some key parameters
    setprop("/autopilot/locks/heading", "");
    setprop("/autopilot/locks/altitude", "");
    setprop("/autopilot/locks/speed", "");
    setprop("/controls/gear/brake-parking", 0);
    setprop("/controls/engines/engine[0]/throttle", 0.0);
    setprop("/controls/engines/engine[1]/throttle", 0.0);
    setprop("/controls/engines/engine[2]/throttle", 0.0);
    setprop("/controls/engines/engine[3]/throttle", 0.0);
    setprop("/controls/flight/elevator", 0.0);
    setprop("/controls/flight/elevator-trim", 0.0);
    setprop("/controls/flight/wing-fold", 0);

    # setup some more parameters
    setprop("/sim/freeze/fuel", 1);
    setprop("/controls/winch/place", 0);
    setprop("/sim/hitches/winch/open", 1);
    setprop("/sim/hitches/winch/winch/rel-speed", 0.0);

    # turn off carrier "ai" so it just drives straight
    setprop("/ai/models/carrier/controls/ai-control", 0);

    # set carrier speed
    setprop("/ai/models/carrier/controls/tgt-speed-kts", carrier_target_speed);

    # for uav demo, tower view is set to the target touch down
    # spot which offers an instructive vantage point (position
    # updated elsewhere)
    setprop("/sim/tower/auto-position", 0);

    # place the winch
    setprop("/controls/winch/place", 1);
}


var update_avior_actuators = func( dt ) {
    var max_zoom_rate = 10*dt;
    var max_pan_rate = 30*dt;
    var max_tilt_rate = 45*dt;

    # winch management (wind us up if we have a locked on winch)
    var winch_open = getprop("/sim/hitches/winch/open");
    if ( winch_open == 0 ) {
	var winch_speed = getprop("/sim/hitches/winch/winch/rel-speed");
	winch_speed += dt * 0.05;
	if ( winch_speed > 0.6 ) {
	    winch_speed = 0.6;
	}
	printf("winch speed = %.1f", winch_speed);
	setprop("/sim/hitches/winch/winch/rel-speed", winch_speed);
    }

    # simulate a bungee line falling off automatically
    var airspeed = getprop("/velocities/airspeed-kt");
    var agl_ft = getprop("/position/altitude-agl-ft");
    if ( airspeed > 50.0 and agl_ft > 10.0 ) {
	# we are airborne
	setprop("/sim/hitches/winch/open", 1);
    }

    var ap_enable = props.globals.getNode("/avior/settings/ap-enable");
    if ( ap_enable.getBoolValue() ) {
	aileron = getprop("/aura-uas/act/aileron");
	if ( aileron != nil ) {
	    setprop( "/controls/flight/aileron", aileron );
	}
	elevator = getprop("/aura-uas/act/elevator");
	if ( elevator != nil ) {
	    setprop( "/controls/flight/elevator", elevator );
	}
	throttle = getprop("/aura-uas/act/throttle");
	if ( throttle != nil ) {
	    setprop( "/controls/engines/engine[0]/throttle", throttle );
	}
	rudder = getprop("/aura-uas/act/rudder");
	if ( rudder != nil and !debug.isnan(rudder) ) {
	    setprop( "/controls/flight/rudder", rudder );
	}
    }

    var turret_enable = props.globals.getNode("/avior/settings/turret-enable");

    if ( (getprop("/sim/current-view/name") == "Camera View")
         and turret_enable.getBoolValue() )
    {
        var target_zoom = getprop("/avior/act/channel6");
        var target_pan = -getprop("/avior/act/channel7");
        if ( target_pan < -180.0 ) { target_pan += 360.0; }
        if ( target_pan > 180.0 ) { target_pan -= 360.0; }
	var target_tilt = -getprop("/avior/act/channel8");
	var cur_zoom = getprop("/sim/current-view/field-of-view");
	var cur_pan = getprop("/sim/current-view/heading-offset-deg");
	var cur_tilt = getprop("/sim/current-view/pitch-offset-deg");
	var diff = 0.0;

	diff = target_zoom - cur_zoom;
	if ( diff > max_zoom_rate ) { diff = max_zoom_rate; }
        if ( diff < -max_zoom_rate ) { diff = -max_zoom_rate; }
        setprop("/sim/current-view/field-of-view", cur_zoom + diff);

	diff = target_pan - cur_pan;
	if ( diff > 180 ) { diff -= 360; }
        if ( diff < -180 ) { diff += 360; }
	if ( diff > max_pan_rate ) { diff = max_pan_rate; }
        if ( diff < -max_pan_rate ) { diff = -max_pan_rate; }
        setprop("/sim/current-view/heading-offset-deg", cur_pan + diff);

	diff = target_tilt - cur_tilt;
	if ( diff > 90 ) { diff = 90; }
        if ( diff < -90 ) { diff = -90; }
	if ( diff > max_tilt_rate ) { diff = max_tilt_rate; }
        if ( diff < -max_tilt_rate ) { diff = -max_tilt_rate; }
        setprop("/sim/current-view/pitch-offset-deg", cur_tilt + diff);
    }
}


var update_avior_autopilot = func() {
    var raw = 0;
    var val = 0;
    var dist_nm = 0;
    var buf = "";
    var len = 0;
    var tmp = 0;
    var hours = 0;
    var mins = 0;
    var secs = 0;

    raw = getprop("/controls/reserved/reserved[0]");
    if ( raw == nil ) {
	raw = 0;
    }      
    val = (raw - 18000.0) / 100;
    setprop("/autopilot/settings/target-bank-deg", val);

    raw = getprop("/controls/reserved/reserved[1]");
    if ( raw == nil ) {
	raw = 0;
    }      
    val = (raw - 9000.0) / 100;
    setprop("/autopilot/settings/target-pitch-deg", val);
    
    raw = getprop("/controls/reserved/reserved[2]");
    if ( raw == nil ) {
	raw = 0;
    }      
    val = (raw - 36000.0) / 100;
    setprop("/avior/heading-bug-offset-deg", val);
    
    raw = getprop("/controls/reserved/reserved[3]");
    if ( raw == nil ) {
	raw = 0;
    }      
    val = (raw - 100000.0) / 1000;
    setprop("/autopilot/internal/target-climb-rate-fps", val);
    
    raw = getprop("/controls/reserved/reserved[4]");
    if ( raw == nil ) {
	raw = 0;
    }      
    val = raw / 100;
    setprop("/autopilot/settings/target-altitude-ft", val);
    
    raw = getprop("/controls/reserved/reserved[5]");
    if ( raw == nil ) {
	raw = 0;
    }      
    val = raw / 100;
    setprop("/autopilot/settings/target-speed-kt", val);

    raw = getprop("/controls/reserved/reserved[6]");
    if ( raw == nil ) {
	raw = 0;
    }      
    val = (raw - 36000.0) / 100;
    setprop("/avior/ground-track-offset-deg", val);

    raw = getprop("/controls/reserved/reserved[7]");
    if ( raw == nil ) {
	raw = 0;
    }
    val = raw * 10.0;
    dist_nm = val * 0.0005399568034557235;
    if ( dist_nm >= 10.0 ) {
	buf = sprintf( "%.1fnm", dist_nm );
    } else if ( dist_nm >= 1.0 ) {
	buf = sprintf( "%.2fnm", dist_nm );
    } else {
	buf = sprintf( "%.0fm", val );
    }
    #len = 6 - size(buf);
    #while ( len > 0 ) {
    #  buf = sprintf(" %s", buf);
    #  len = len - 2;
    #}
    setprop("/avior/dist-string", buf);

    raw = getprop("/controls/reserved/reserved[8]");
    if ( raw == nil ) {
	raw = 0;
    }
    tmp = raw;
    hours = int(raw/3600.0);
    tmp = tmp - hours * 3600.0;
    mins = int(tmp/60.0);
    tmp = tmp - mins*60.0;
    secs = tmp;
    if ( hours >= 1 ) {
	buf = sprintf("%d:%02d:%02dh", hours, mins, secs );
    } else if ( mins >= 1 ) {
	buf = sprintf("%d:%02dm", mins, secs );
    } else {
	buf = sprintf("%.0fs", secs );
    }
    setprop("/avior/eta-string", buf);
}


# some temp/testing code to attempt to validate the "fluff" numbers
# yasim outputs
var last_ve = 0;
var last_vn = 0;
var last_vd = 0;
var total_time = 0;

var validate_physics = func( dt ) {

    if ( dt < 0.0001 ) { return; } # bail on zero dt

    total_time += dt;

    var ve = getprop("/velocities/speed-east-fps");
    var vn = getprop("/velocities/speed-north-fps");
    var vd = getprop("/velocities/speed-down-fps");

    var accele = (ve - last_ve) / dt;
    var acceln = (vn - last_vn) / dt;
    var acceld = (vd - last_vd) / dt;

    var off_accele = getprop("/accelerations/ned/east-accel-fps_sec");
    var off_acceln = getprop("/accelerations/ned/north-accel-fps_sec");
    var off_acceld = getprop("/accelerations/ned/down-accel-fps_sec");

    var time = getprop("/sim/time/elapsed-sec");

    #printf("%.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f",
    #	   time, total_time, dt, accele, acceln, acceld, off_accele, off_acceln, off_acceld);

    last_ve = ve;
    last_vn = vn;
    last_vd = vd;
}


var avior_main = func {
    var nasal_control = getprop("/uas/state");
    if ( nasal_control == "" ) {
        time = getprop("/sim/time/elapsed-sec");
        var dt = time - avior_last_time;
        avior_last_time = time;

	# temp/test
	validate_physics( dt );

        update_avior_actuators( dt );
        update_avior_autopilot( dt );

    }

    settimer(avior_main, 0);
}

setlistener("/sim/signals/fdm-initialized",
            func {
                avior_main();
            });
