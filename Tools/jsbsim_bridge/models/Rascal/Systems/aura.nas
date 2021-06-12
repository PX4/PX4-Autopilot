var update_aura = func( dt ) {
    var max_zoom_rate = 10*dt;
    var max_pan_rate = 30*dt;
    var max_tilt_rate = 45*dt;

    var ap_enable = props.globals.getNode("/aura-uas/settings/ap-enable");
    if ( ap_enable == nil ) {
	props.globals.initNode("/aura-uas/settings/ap-enable", 0, "BOOL", 1);
        ap_enable = props.globals.getNode("/aura-uas/settings/ap-enable");
    }
    if ( ap_enable.getBoolValue() ) {
	setprop( "/controls/flight/aileron", getprop("/aura-uas/act/aileron") );
	setprop( "/controls/flight/elevator", getprop("/aura-uas/act/elevator") );
	setprop( "/controls/flight/rudder", getprop("/aura-uas/act/rudder") );
	setprop( "/controls/engines/engine/throttle", getprop("/aura-uas/act/throttle") );
    }

    var turret_enable = props.globals.getNode("/aura-uas/settings/turret-enable");
    if ( turret_enable == nil ) {
	props.globals.initNode("/aura-uas/settings/turret-enable", 0, "BOOL", 1);
        turret_enable = props.globals.getNode("/aura-uas/settings/turret-enable");
    }

    if ( (getprop("/sim/current-view/name") == "Camera View")
         and turret_enable.getBoolValue() )
    {
        var target_zoom = getprop("/aura-uas/act/channel6");
        var target_pan = -getprop("/aura-uas/act/channel7");
        if ( target_pan < -180.0 ) { target_pan += 360.0; }
        if ( target_pan > 180.0 ) { target_pan -= 360.0; }
	var target_tilt = -getprop("/aura-uas/act/channel8");
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
