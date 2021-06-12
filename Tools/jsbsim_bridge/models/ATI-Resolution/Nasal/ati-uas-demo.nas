# script mode (carrier versus runway)
#var mode = "carrier";
var mode = "runway";

#var launch_mode = "power";
var launch_mode = "winch";

var enable_tanker = 0;
var scene_created = 0;

# landing configuration parameters
var gate_dist_nm = 0.25;	# nm
var glideslope = 3.0; 		# degrees
var tz_offset = 40;
if ( mode == "carrier" ) {
    glideslope = 3.5;
}
var rotate_speed = 20;
var climbout_alt_ft = 200;
var climbout_speed_kt = 35;
var operational_alt = 500;
var min_speed = 30;
var route_speed = 40;
var max_speed = 60;
var pattern_height_ft = 200;
var flap_speed_kt = 35;
var downwind_speed_kt = 40;
var approach_max_decent_rate = -16.7; # -1000 fpm
var approach_max_climb_rate = 8.333; # 500 fpm
var base_speed_kt = 35;
var gear_down_speed_kt = 35;
var final_speed_kt = min_speed;
var flare_gain = 2;		# increase value to flare sooner and more gently
var max_pattern_bank = 30;
var xtrack_route_gain = 1.0;
var xtrack_downwind_gain = 1.0;
var xtrack_final_gain = 1.0;

# circling parameters
var circle_speed = downwind_speed_kt; #kts
var circle_bank = 15;		      #degrees

# carrier configuration
var carrier_target_speed = 0.0;
var carrier_deck_alt = 75.9;	# hard coded in case someone tries to
				# init in the air we don't want this
				# reset

# internal variables
var time = 0.0;
var timer_start = 0.0;
var last_time = 0.0;
var speedbrake_filter = 0.0;
var last_uas_view = "";

# variables to track carrier heading and speed (probably the self
# reported numbers would actually be good enough??? worth another test
# at some point)
var cc_timer = 0.0;
var carrier_heading = 0.0;
var carrier_speed = 0.0;
var last_carrier_coord = geo.aircraft_position(); # temp init

# variables to track tanker heading and speed
var tanker_timer = 0.0;
var tanker_heading = 0.0;
var tanker_speed = 0.0;
var tanker_coord = geo.aircraft_position(); # temp init
var last_tanker_coord = geo.aircraft_position(); # temp init
var refuel_state = 0;

# variables to auto snap camera for completely coverage
var last_camera_coord = geo.aircraft_position();

# variables to compute our own instantaneous ground speed
var own_timer = 0.0;
var own_groundspeed = 0.0;
var own_coord = geo.aircraft_position();
var last_own_coord = geo.aircraft_position();

# runway variables
var home_apt = nil;
var rwy_coord = geo.aircraft_position(); # temp init
var rwy_heading = 0.0;
var rwy_recip = 0.0;
var xtrack = 0.0;		# cross track error (m)
var rwy_dist = 0.0;		# distance along runway center line (m)
var last_rwy_dist = 0.0;
var closing_speed_kt = 0.0;
var rwy_gate = geo.aircraft_position(); # temp init
var rwy_course_to = 0;
var rwy_distance_to = 0;
var real_glideslope = 3.0;

# approach variables
var approach_diameter = 0.0;
var approach_45_dist = 0.0;
var approach_dist_to_touchdown = 0.0;
var last_dist_to_touchdown = 0.0;
var approach_dw_alt = 0.0;
var approach_ideal_alt = 0.0;
var approach_alt_error = 0.0;
var approach_lock_glideslope = 0;
var approach_vertspeed_fps = 0;

var circle_coord = geo.aircraft_position(); # temp init

var c1 = geo.aircraft_position();
var c2 = geo.aircraft_position();
c1.set_lon(-123.6618751);
c1.set_lat(37.81747044);
c2.set_lon(-123.6627194);
c2.set_lat(37.81862305);
print("HERE: crs = ", c2.course_to(c1), " dist = ", c2.distance_to(c1) );

var create_scene_if_needed = func() {
    var scene_lon = -123.1;
    var scene_lat = 37.7;

    if ( scene_created ) {
	return;
    }

    var scene_elev = geo.elevation(scene_lat, scene_lon);
    if ( scene_elev != nil ) {
	var base = geo.aircraft_position();
	base.set_lon(scene_lon);
	base.set_lat(scene_lat);

	# randomize scene location just a bit
	base.apply_course_distance( 360.0 * rand(), 1500 + 1000*rand() );
	geo.put_model("Models/Aircraft/A380.xml",
		      base.lat(), base.lon(), scene_elev - (11.0 * .3048),
		      360.0 * rand(), 0, -5);

	# scatter several small life rafts near by
	for ( var i = 0; i < 9; i += 1 ) {
	    var lr = geo.Coord.new( base );
	    lr.apply_course_distance( 360.0 * rand(), 50 + 250*rand() );
	    geo.put_model("Models/Maritime/Misc/liferaft4m.ac",
			  lr.lat(), lr.lon(), scene_elev, 0, 0, -5);
	}

	# scatter a couple larger life rafts near by
	for ( var i = 0; i < 3; i += 1 ) {
	    var lr = geo.Coord.new( base );
	    lr.apply_course_distance( 360.0 * rand(), 50 + 250*rand() );
	    geo.put_model("Models/Maritime/Misc/liferaft8m.ac",
			  lr.lat(), lr.lon(), scene_elev,
			  8*rand() - 4, 8*rand() - 4 , 8*rand() - 4);
	}

	scene_created = 1;
    }
}


var update_view = func() {
    var current_view = getprop("/sim/current-view/view-number");
    var uas_view = getprop("/uas/view-mode");

    if ( uas_view == "" or uas_view == nil ) {
	setprop("/uas/view-mode", "Gyro Camera");
	setprop("/uas/camera-zoom", 1.0);
    }

    if ( uas_view == last_uas_view ) {
	# nothing actually changed, probably just the dialog box refreshin
	return;
    }
    last_uas_view = uas_view;

    var view_num = 0;
    if ( uas_view == "Cockpit" ) {
	view_num = 0;
    } elsif ( uas_view == "Gyro Camera" ) {
	view_num = view.indexof(uas_view);
    } elsif( uas_view == "External" ) {
	view_num = 2;
    } elsif ( uas_view == "Fly By" ) {
	view_num = 6;
    } elsif ( uas_view == "PLAT Camera" ) {
	view_num = 4;
    }

    if ( current_view != view_num ) {
	setprop("/sim/current-view/view-number", view_num);
    }

    # manage hud configuration per view
    if ( uas_view == "Cockpit" ) {
	setprop("/sim/hud/enable3d[1]", 1); # conformal hud for cockpit view
    } elsif ( uas_view == "Gyro Camera" ) {
	setprop("/sim/hud/enable3d[1]", 0); # non-conformal hud for cam view
    } elsif ( uas_view == "External" ) {
	setprop("/sim/hud/enable3d[1]", 1); # conformal hud for chase view
    } elsif ( uas_view == "Fly By" ) {
	setprop("/sim/hud/enable3d[1]", 0); # non-conformal hud for fly-by view
    } elsif ( uas_view == "PLAT Camera" ) {
	setprop("/sim/hud/enable3d[1]", 0); # non-conformal hud for deck cam
    } else {
	#setprop("/sim/hud/visibility[1]", 0); # hud off for vanity views
    }
}

var update_zoom = func() {
    # set zoom from gui if camera view
    var zoom = getprop("/uas/camera-zoom");
    if ( !zoom ) { 
	zoom = 1.0; setprop("/uas/camera-zoom", 1.0);
    }
    var current_view = getprop("/sim/current-view/view-number");
    if ( current_view == view.indexof("Gyro Camera") ) {
	setprop("/sim/current-view/field-of-view", 55 / zoom );
    }
}

var update_camera_target = func() {
    var cam_target = getprop("/uas/camera-target");
    if ( cam_target == "" or cam_target == nil ) {
	setprop("/uas/camera-target", "Carrier");
    }

    if ( cam_target == "Next Waypoint" ) {
	var routesize = getprop("/autopilot/route-manager/route/num");
	var cur_wp = getprop("/autopilot/route-manager/current-wp");
	if ( cur_wp >= 0 and cur_wp < routesize ) {
	    var routeNode
		= props.globals.getNode("/autopilot/route-manager/route");
	    var wpNode = routeNode.getChild("wp", cur_wp);
	    if ( wpNode != nil ) {
		var lon = wpNode.getChild("longitude-deg").getValue();
		var lat = wpNode.getChild("latitude-deg").getValue();
		setprop("/sim/input/click/longitude-deg", lon);
		setprop("/sim/input/click/latitude-deg", lat);
		var scene_elev = geo.elevation(lat, lon);
		if ( scene_elev != nil ) {
		    setprop("/sim/input/click/altitude-ft", scene_elev*M2FT);
		}
	    }
	}
    } elsif ( cam_target == "Carrier" ) {
	setprop("/sim/input/click/longitude-deg", rwy_coord.lon());
	setprop("/sim/input/click/latitude-deg", rwy_coord.lat());
	setprop("/sim/input/click/elevation-ft", carrier_deck_alt - 10);
    }
}


var update_camera_trigger = func() {
    var enable = getprop("/uas/camera-trigger");
    if ( enable == nil or !enable ) {
        return;
    }
    var overlap = 0.25;
    var agl_m = getprop("/position/altitude-agl-ft") * FT2M;
    var fov = getprop("/sim/current-view/field-of-view");
    var fov2 = fov * 0.5;
    var hdist = math.tan(fov2*D2R) * agl_m * 2.0;

    var cur_coord = geo.aircraft_position();
    var dist = last_camera_coord.distance_to( cur_coord );

    if ( (dist >= hdist * (1.0 - overlap)) and (agl_m > 75) ) {
        last_camera_coord.set( cur_coord );
        fgcommand("screen-capture");
    }
}

var elevator_trim = 0.0;
var elevator_time = 0.0;
var trim_hz = 3.0;
var trim_gain = 0.01;
var airspeed_noise = 0.1;
var emulate_pitch_bobble = func(dt) {
    var airspeed = getprop("/velocities/airspeed-kt");

    var do_bobble = getprop("/uas/emulate-pitch-bobble");
    if ( do_bobble ) {
	elevator_time += dt;
	var trim = math.sin(elevator_time*math.pi*2*trim_hz)
	    * trim_gain * rand();
	setprop("/controls/flight/elevator-trim", trim);
	airspeed += airspeed_noise * (2.0*rand()-1.0);
	#print("airspeed = ", airspeed);
    }
    setprop("/velocities/airspeed-noisy-kt", airspeed);
}

var normdeg = func(a) {
    while (a >= 180)
	a -= 360;
    while (a < -180)
	a += 360;
    return a;
}

var best_runway = func(apt) {
    var wind_speed = getprop("/environment/wind-speed-kt");
    var wind_from = wind_speed ? getprop("/environment/wind-from-heading-deg") : 270;
    var min = 361;
    var rwy = nil;

    foreach (var r; keys(apt.runways)) {
	var curr = apt.runways[r];
	var deviation = math.abs(normdeg(wind_from - curr.heading));
	if ( deviation < min ) {
	    min = deviation;
	    rwy = curr;
	}
	#var v = (0.01 * curr.length + 0.01 * curr.width) / deviation;
	#if (v > max) {
	#    max = v;
	#    rwy = curr;
	#}
    }
    return rwy;
}

# set the runway location and heading manually (used then for
# computing xtrack error, closing speed, etc.)
var set_runway_location_by_ident = func( apt_id = nil ) {

    if ( apt_id ) {
	home_apt = airportinfo( apt_id );
    } else {
	home_apt = airportinfo();
    }

    var rwy = best_runway( home_apt );

    setprop("/uas/airport-id", home_apt.id);
    setprop("/uas/runway-id", rwy.id);

    var m = geo.Coord.new().set_latlon(rwy.lat, rwy.lon);
    # m.apply_course_distance(rwy.heading + 180, rwy.length / 2 - rwy.threshold - tz_offset);
    print("rwy.length = ", rwy.length);
    #m.apply_course_distance(rwy.heading + 180, rwy.length / 2);

    var g = geodinfo(m.lat(), m.lon());
    m.set_alt( g != nil ? g[0] : apt.elevation );

    rwy_coord.set_lon( m.lon() );
    rwy_coord.set_lat( m.lat() );
    rwy_coord.set_alt( m.alt() );

    rwy_heading = rwy.heading;

    rwy_recip = rwy_heading + 180.0;
    if ( rwy_recip > 360.0 ) {
	rwy_recip -= 360.0;
    }
}

# set the runway location and heading manually (used then for
# computing xtrack error, closing speed, etc.)
var set_runway_location_by_coord = func(lon_deg, lat_deg, alt_m, heading_deg) {
    rwy_coord.set_lon( lon_deg );
    rwy_coord.set_lat( lat_deg );
    rwy_coord.set_alt( alt_m );

    rwy_heading = heading_deg;

    rwy_recip = rwy_heading + 180.0;
    if ( rwy_recip > 360.0 ) {
	rwy_recip -= 360.0;
    }
}

# set the runway gate position ... this is a reference point for
# setting up a visual approach to a specific touchdown point and
# heading
var set_runway_gate = func( gate_dist_nm, glideslope_deg ) {
    rwy_gate.set( rwy_coord );
    rwy_gate.apply_course_distance( rwy_recip, gate_dist_nm*NM2M );
    var alt_offset = math.sin(glideslope_deg*D2R) * gate_dist_nm*NM2M;
    rwy_gate.set_alt( rwy_coord.alt() + alt_offset );
}

# compute cross track error and runway distance.
var calc_xtrack = func() {
    var cur_coord = geo.aircraft_position();
    #rwy_coord.dump();
    #cur_coord.dump();
    rwy_course_to = rwy_coord.course_to( cur_coord );
    rwy_distance_to = rwy_coord.distance_to( cur_coord );
    var angle = rwy_course_to - rwy_heading;
    if ( angle < -180 ) {
	angle += 360.0;
    } elsif ( angle > 180 ) {
	angle -= 360.0;
    }
    xtrack = math.sin( angle * D2R ) * rwy_distance_to;
    rwy_dist = math.cos( angle * D2R ) * rwy_distance_to;
    setprop("/uas/xtrack", xtrack);
    setprop("/uas/rwy-dist", rwy_dist);
    #printf("dist=%.0f brg=%.1f xtrack=%.1f", distance, bearing, xtrack);
}

# compute basic heading and distance to runway
var basic_geo_info = func() {
}


# given the current wind vector and a target ground course, compute
# the required (true) heading
var wind_course = func( course_deg, truespeed, add_carrier_motion ) {
    var result = {};
    var heading = course_deg;
    var groundspeed = 0;
    var course_rad = course_deg * D2R;

    var winddir = getprop("/environment/wind-from-heading-deg") * D2R;
    var windspeed = getprop("/environment/wind-speed-kt");

    if ( add_carrier_motion ) {
	var angle = 0;
	var mps = 0;

	# compute wind vector components
	angle = 0.5*math.pi - winddir;
	mps = windspeed * KT2MPS;
	var we = mps * math.cos(angle);
	var wn = mps * math.sin(angle);
	#print("we = ", we, " wn = ", wn);

	# add carrier motion into "effective" wind vector
	#var carrier_heading
	#    = getprop("/ai/models/carrier/orientation/true-heading-deg");
	#var carrier_speed
	#    = getprop("/ai/models/carrier/velocities/speed-kts");
	angle = (90 - carrier_heading) * D2R;
	mps = carrier_speed * KT2MPS;
	var ce = mps * math.cos(angle);
	var cn = mps * math.sin(angle);
	#print("ce = ", ce, " cn = ", cn);

	var te = we + ce;
	var tn = wn + cn;

	# hack winddir and speed to include carrier motion
	winddir = 0.5*math.pi - math.atan2(tn, te);
	windspeed = math.sqrt( tn*tn + te*te ) * MPS2KT;
	#print("te = ", te, " tn = ", tn);
	#print("dir = ", winddir*R2D, " speed = ", windspeed);
    }

    var swc = (windspeed/truespeed) * math.sin(winddir - course_rad);
    if ( math.abs(swc) > 1.0 ) {
	# course cannot be flown, wind too strong
    } else {
	heading = ( course_rad + math.asin(swc) ) * R2D;
	if ( heading < 0 ) { heading += 360.0; }
	if ( heading > 360 ) { heading -= 360.0; }
	groundspeed = truespeed * math.sqrt(1-swc*swc)
	    - windspeed * math.cos(winddir - course_rad);
	if (groundspeed < 0) {
	    # course cannot be flown-- wind too strong
	}
    }

    result.heading = heading;
    result.groundspeed = groundspeed;

    return result;
}


var manage_speedbrake = func( target_speed, slop ) {
    var own_indspeed
	= getprop("/instrumentation/airspeed-indicator/indicated-speed-kt");
    var overspeed = own_indspeed - (target_speed + slop);
    var speedbrake = 0.0;
    if ( overspeed > 0.0 ) {
	speedbrake = overspeed / 20;
	if ( speedbrake > 1.0 ) { speedbrake = 1.0; }
    } else {
	speedbrake = 0.0;
    }
    speedbrake_filter = 0.95 * speedbrake_filter + 0.05 * speedbrake;
    setprop("/controls/flight/speedbrake", speedbrake_filter);
}


var compute_approach_metrics = func( dt ) {
    var state = getprop("/uas/state");

    var airspeed = getprop("/velocities/airspeed-kt");

    # compute turning diameter for target downwind speed, if we
    # can't get there physically, too bad!
    var radius_ft = (downwind_speed_kt*downwind_speed_kt)
	/ (11.23*math.tan(0.01745*max_pattern_bank));
    var radius_m = radius_ft * FT2M;
    approach_diameter = 2.0 * radius_m;
    var approach_radius = approach_diameter * 0.5;
    setprop("/uas/approach/diameter", approach_diameter);

    approach_45_dist = approach_diameter * 1.414; # sqrt(2)
    setprop("/uas/approach/forty-five-dist", approach_45_dist);

    # degrees off from approach heading
    var truespeed = getprop("/instrumentation/airspeed-indicator/true-speed-kt");
    var windtri = wind_course( rwy_heading, truespeed, (mode == "carrier") );
    var current_heading = getprop("/orientation/heading-deg");
    var turn_deg = windtri.heading - current_heading;
    if ( turn_deg < -180 ) { turn_deg += 360.0; }
    if ( turn_deg > 180 ) { turn_deg -= 360.0; }

    var turn_rad = math.abs(turn_deg) * D2R;
    setprop("/uas/approach/runway-heading-error", turn_deg);

    # distance to fly the remaining turn radius back to the runway heading
    var turn_dist_m = turn_rad * approach_radius * 1.5;
    var turn_dist_180_m = math.pi * approach_radius;
    setprop("/uas/approach/turn-dist", turn_dist_m);

    # distance to turn to base
    var dist_to_turn = rwy_dist + approach_45_dist;

    approach_dist_to_touchdown = 0;
    if ( state == "downwind" ) {
	# compute approx horizontal distance to touchdown
	if ( dist_to_turn > 0 ) {
	    # short of base turn point

	    # feather our turn distance down to zero as we approach
	    # the threshold
	    var factor = (approach_45_dist - dist_to_turn) / approach_45_dist;
	    if ( factor > 1.0 ) { factor = 1.0; }
	    if ( factor < 0.0 ) { factor = 0.0; }
	    #var turn_diff = turn_dist_180_m - turn_dist_m;
	    approach_dist_to_touchdown = dist_to_turn + approach_45_dist
		+ turn_dist_180_m*(1-factor) + turn_dist_m*factor - math.sin(turn_rad)*approach_radius*factor;
	} else {
	    # past base turn point
	    approach_dist_to_touchdown = -rwy_dist + turn_dist_m
		- math.sin(turn_rad)*approach_radius;
	}
    } elsif ( state == "base" ) {
	approach_dist_to_touchdown = -rwy_dist + turn_dist_m
	    - math.sin(turn_rad)*approach_radius;
    } elsif ( state == "final" ) {
	# feather our turn distance down to zero as we approach the threshold
	var factor = -rwy_dist / approach_45_dist;
	if ( factor > 1.0 ) { factor = 1.0; }
	if ( factor < 0.0 ) { factor = 0.0; }
	approach_dist_to_touchdown = -rwy_dist + (turn_dist_m
	    - math.sin(turn_rad)*approach_radius) * factor;
    }
    # printf("dist to touchdown = %.2f\n", approach_dist_to_touchdown);
    setprop("/uas/approach/dist-to-touchdown", approach_dist_to_touchdown);

    var groundspeed_kt = getprop("/velocities/groundspeed-kt");

    # compute closing speed on touchdown point (keep as positive speed
    # so we don't decend when we want to climb.)
    var dist = math.abs(last_rwy_dist - rwy_dist);
    last_rwy_dist = rwy_dist;
    var mps = dist / dt;
    closing_speed_kt = 0.05 * closing_speed_kt + 0.95 * mps * MPS2KT; # option a (good for carrier)
    # closing_speed_kt = 0.05 * closing_speed_kt + 0.95 * groundspeed_kt; # option b (good for runways?)
    setprop("/uas/approach/closing-speed-kt", closing_speed_kt);

    # compute optimal decent rate for our target airspeed in zero wind
    var approach_tri = wind_course( rwy_heading, final_speed_kt, 0 );
    var optimal_decent_kt = math.sin(glideslope*D2R) * final_speed_kt;
    var ratio = optimal_decent_kt / approach_tri.groundspeed;
    if ( ratio > 1.0 ) { ratio = 1.0; }
    if ( ratio < 0.0 ) { ratio = 0.0; }
    real_glideslope = math.asin( ratio ) * R2D;
    setprop("/uas/approach/real-glideslope", real_glideslope);

    # compute optimal decent rate for our closing speed to be on
    # the glide slope
    var vertspeed_kt = -math.sin(real_glideslope*D2R) * closing_speed_kt;
    var vertspeed_mps = vertspeed_kt * KT2MPS;
    approach_vertspeed_fps = vertspeed_mps * M2FT;
    setprop("/uas/approach/ideal-vertical-rate", approach_vertspeed_fps);

    # compute our ideal altitude
    approach_ideal_alt = (rwy_coord.alt()
			  + math.tan(real_glideslope*D2R)
			  * approach_dist_to_touchdown) * M2FT;
    setprop("/uas/approach/ideal-alt-ft", approach_ideal_alt);

    # compute our altitude error
    var cur_alt = getprop("/position/altitude-ft");
    approach_alt_error = approach_ideal_alt - cur_alt;
    setprop("/uas/approach/vert-error", approach_alt_error);
}


var task_circle_current_pos = func() {
    var lon = getprop("/position/longitude-deg");
    var lat = getprop("/position/latitude-deg");
    var ground = getprop("/position/ground-elev-ft");
    setprop("/sim/input/click/longitude-deg", lon);
    setprop("/sim/input/click/latitude-deg", lat);
    setprop("/sim/input/click/elevation-ft", ground );
    setprop("/uas/state", "circle");
}


var task_go_home = func() {
    setprop("/uas/state", "gohome");
    set_runway_location_by_ident( getprop("/uas/airport-id") );
    set_runway_gate( gate_dist_nm, glideslope );
}


var task_nasal_launch = func() {
    # disable external avior control
    setprop("/avior/settings/ap-enable", 0);
    setprop("/avior/settings/turret-enable", 0);

    # enable internal nasal autopilot system
    setprop("/uas/master-switch", 1);

    # setup initial state of some key parameters
    setprop("/autopilot/locks/heading", "");
    setprop("/autopilot/locks/altitude", "");
    setprop("/autopilot/locks/speed", "");
    setprop("/controls/gear/brake-parking", 1);
    setprop("/controls/engines/engine[0]/throttle", 0.0);
    setprop("/controls/engines/engine[1]/throttle", 0.0);
    setprop("/controls/engines/engine[2]/throttle", 0.0);
    setprop("/controls/engines/engine[3]/throttle", 0.0);
    setprop("/controls/flight/elevator", 0.0);
    setprop("/controls/flight/elevator-trim", 0.0);

    # switch to launch mode
    setprop("/uas/state", "launch-init");
    print("state -> launch-init");
}


var update_state = func( dt ) {
    if ( dt < 0.000001 ) {
	return;
    }

    var state = getprop("/uas/state");

    create_scene_if_needed();
    update_zoom();
    update_camera_target();
    update_camera_trigger();
    emulate_pitch_bobble(dt);

    if ( mode == "carrier" ) {
	# update runway postion and heading from carrier position
	var lon = getprop("/ai/models/carrier/position/longitude-deg");
	var lat = getprop("/ai/models/carrier/position/latitude-deg");
	var carrier_coord = geo.aircraft_position(); # temp init
	carrier_coord.set_lon( lon );
	carrier_coord.set_lat( lat );
	carrier_coord.set_alt( carrier_deck_alt*FT2M );

	# debug
	#var cur_coord = geo.aircraft_position();
	#var debug_course_to = carrier_coord.course_to( cur_coord );
	#var debug_distance_to = carrier_coord.distance_to( cur_coord );
	#setprop("/uas/a-course", debug_course_to);
	#setprop("/uas/a-distance", debug_distance_to);

	var carrier_yaw
	    = getprop("/ai/models/carrier/orientation/true-heading-deg");
	var deck_heading = carrier_yaw - 8;
	if ( deck_heading < 0 ) {
	    deck_heading += 360.0;
	}

	var td_offset_hdg = carrier_yaw - 178.283;
	if ( td_offset_hdg < 0 ) {
	    td_offset_hdg += 360.0;
	}
	var td_offset_dist = 151.44;
	carrier_coord.apply_course_distance( td_offset_hdg, td_offset_dist );

	set_runway_location_by_coord( carrier_coord.lon(), carrier_coord.lat(),
				      carrier_deck_alt*FT2M, deck_heading );
	set_runway_gate( gate_dist_nm, glideslope );

	# estimate carrier heading (because the AI system doesn't move
	# it exactly at the correct heading ... doh!
	cc_timer += dt;
	if ( cc_timer > 10 ) {
	    carrier_heading = last_carrier_coord.course_to( rwy_coord );
	    var dist = last_carrier_coord.distance_to( rwy_coord );
	    var mps = dist / cc_timer;
	    carrier_speed = mps * MPS2KT;
	    last_carrier_coord.set(rwy_coord);
	    cc_timer = 0.0;
	}
	#last_carrier_coord.dump();
	#rwy_coord.dump();
	#print("rwyhdg = ", rwy_heading, " chdg = ", carrier_heading, " speed = ", carrier_speed);
    }

    # set 'tower' view paramters
    setprop("/sim/tower/longitude-deg", rwy_coord.lon());
    setprop("/sim/tower/latitude-deg", rwy_coord.lat());
    if ( mode == "carrier" ) {
	setprop("/sim/tower/altitude-ft", rwy_coord.alt()*M2FT - 8);
	if ( getprop("/sim/current-view/view-number") == 4 ) {
	    setprop("/sim/current-view/goal-pitch-offset-deg", 3.5);
	    setprop("/sim/current-view/goal-heading-offset-deg",
		    carrier_yaw - 22);
	}
    } else {
	setprop("/sim/tower/altitude-ft", rwy_coord.alt()*M2FT + 2);
        setprop("/sim/tower/heading-deg", rwy_recip);
	if ( getprop("/sim/current-view/view-number") == 4 ) {
	    setprop("/sim/current-view/goal-pitch-offset-deg", real_glideslope);
	    # setprop("/sim/current-view/goal-heading-offset-deg", 180.0);
	}
    }
	
    if ( enable_tanker ) {
	var have_tanker = props.globals.getNode("/ai/models/tanker");
	if ( have_tanker == nil ) {
	    tanker.request();	   # mostly out in front heading our way
	    #tanker.request_random();  # random heading and starting point
	} else {
	    # estimate tanker heading (because the AI system doesn't move
	    # it exactly at the correct heading ... doh!
	    var tanker_lon = getprop("/ai/models/tanker/position/longitude-deg");
	    var tanker_lat = getprop("/ai/models/tanker/position/latitude-deg");
	    var tanker_alt = getprop("/ai/models/tanker/position/altitude-ft");
	    tanker_coord.set_lon( tanker_lon );
	    tanker_coord.set_lat( tanker_lat );
	    tanker_coord.set_alt( tanker_alt );
	    tanker_timer += dt;
	    if ( tanker_timer > 1.0 ) {
		tanker_heading = last_tanker_coord.course_to( tanker_coord );
		var dist = last_tanker_coord.distance_to( tanker_coord );
		var mps = dist / tanker_timer;
		var new_gs = mps * MPS2KT;
		tanker_speed = 0.75 * tanker_speed + 0.25 * new_gs;
		last_tanker_coord.set(tanker_coord);
		tanker_timer = 0.0;
	    }
	}
    }

    # compute our own instantaneous ground speed
    own_coord = geo.aircraft_position();
    own_timer += dt;
    if ( own_timer > 0.0001 ) {
	var dist = last_own_coord.distance_to( own_coord );
	var mps = dist / own_timer;
	var new_gs = mps * MPS2KT;
	own_groundspeed = 0.9 * own_groundspeed + 0.1 * new_gs;
	last_own_coord.set(own_coord);
	own_timer = 0.0;
    }
 
    calc_xtrack();

    if ( state == "init-startup" ) {
	setprop("/autopilot/locks/heading", "");
	setprop("/autopilot/locks/altitude", "");
	setprop("/autopilot/locks/speed", "");
	setprop("/controls/gear/brake-parking", 1);
	setprop("/controls/engines/engine[0]/throttle", 0.0);
	setprop("/controls/engines/engine[1]/throttle", 0.0);
	setprop("/controls/engines/engine[2]/throttle", 0.0);
	setprop("/controls/engines/engine[3]/throttle", 0.0);
	setprop("/controls/flight/elevator", 0.0);
	setprop("/controls/flight/elevator-trim", 0.0);
	setprop("/uas/state", "ready");
	print("state -> ready");
   } elsif ( state == "launch-init" ) {
        print("Initializing UAS auto-launch system.");
	setprop("/sim/freeze/fuel", 1);
	setprop("/controls/gear/brake-parking", 1);
	setprop("/controls/winch/place", 0);
	setprop("/sim/hitches/winch/open", 1);
	setprop("/sim/hitches/winch/winch/rel-speed", 0.0);
	var coord = geo.aircraft_position();
	var heading = getprop("/orientation/heading-deg");
	#set_runway_location_by_coord( coord.lon(), coord.lat(), coord.alt(), heading );
	set_runway_location_by_ident();
	set_runway_gate( gate_dist_nm, glideslope );

	setprop("/autopilot/settings/target-yaw-deg", heading);
	# setprop("/uas/view-mode", "Fly By");
	
	# turn off carrier "ai" so it just drives straight
	setprop("/ai/models/carrier/controls/ai-control", 0);

	# set carrier speed
	setprop("/ai/models/carrier/controls/tgt-speed-kts",
		carrier_target_speed);

	# for uav demo, tower view is set to the target touch down
	# spot which offers an instructive vantage point (position
	# updated elsewhere)
	setprop("/sim/tower/auto-position", 0);

	approach_lock_glideslope = 0;
	refuel_state = 0;

	setprop("/uas/state", "launch-init-settle");
	print("state -> launch-init-settle");
    } elsif ( state == "launch-init-settle" ) {
	# give a few seconds for the sim to settle
	var time = getprop("/sim/time/elapsed-sec");
	if ( time > 10.0 ) {
	    carrier_deck_alt = getprop("/position/altitude-ft");
	    setprop("/uas/state", "launch-pretakeoff");
	    print("state -> launch-pretakeoff");
	    # turn on wing leveler
	    setprop("/autopilot/locks/heading", "bank-hold");
	    setprop("/autopilot/settings/target-bank-deg", 0);
	    # turn on pitch hold (set to ground attitude for takeoff run)
	    #setprop("/autopilot/locks/altitude", "pitch-hold");
	    #var pitch = getprop("/orientation/pitch-deg");
	    #setprop("/autopilot/settings/target-pitch-deg", pitch + 1);
	    setprop("/autopilot/locks/altitude", "throttle");
	    var target_alt
		= int((carrier_deck_alt + operational_alt) / 100) * 100;
	    setprop("/autopilot/settings/target-altitude-ft", target_alt);
	    setprop("/uas/flight-altitude-ft", target_alt);
	}
    } elsif ( state == "launch-pretakeoff" ) {
	setprop("/controls/flight/wing-fold", 0);
	if ( mode == "carrier" ) {
	    setprop("/controls/gear/launchbar", 1);
	}
	var wingpos = getprop("/canopy/position-norm");
	if ( wingpos == nil ) { wingpos = 0.0; }
	if ( wingpos < 0.0001 ) {
	    setprop("/controls/flight/flaps", 0.36);
	    setprop("/controls/flight/flapscommand", 1);
	    setprop("/uas/state", "launch-takeoff");
	    print("state -> launch-takeoff");
	}
    } elsif ( state == "launch-takeoff" ) {
	if ( launch_mode == "winch" ) {
	    setprop("/autopilot/locks/altitude", "throttle");
	    setprop("/autopilot/locks/speed", "elevator");
	    setprop("/controls/winch/place", 1);
	    setprop("/uas/state", "launch-takeoff-winch");
	    print("state -> launch-takeoff-winch");
	}

	# advance throttle to full over 2 seconds
	var throttle = getprop("/controls/engines/engine[0]/throttle");
	throttle += dt * 0.5;
	if ( throttle > 1.0 ) {
	    throttle = 1.0;
	}
	setprop("/controls/engines/engine[0]/throttle", throttle);
	setprop("/controls/engines/engine[1]/throttle", throttle);
	setprop("/controls/engines/engine[2]/throttle", throttle);
	setprop("/controls/engines/engine[3]/throttle", throttle);

	# release parking brake after engines have started to spool up
	if ( throttle > 0.5 ) {
	    setprop("/controls/gear/brake-parking", 0);
	}

	var airspeed = getprop("/velocities/airspeed-kt");

	if ( mode == "carrier" ) {
	    # once engines spool up, set climbout pitch and launch!
	    var n1 = getprop("/engines/engine[0]/n1");
	    if ( n1 >= 100 ) {
		setprop("/controls/gear/catapult-launch-cmd", 1);
	    }
	} elsif ( mode == "runway" ) {
	    var groundspeed = getprop("/velocities/groundspeed-kt");
	    if ( groundspeed > 1.0 ) {
		# once we are rolling, turn on steering hold (set to
		# current heading for t-o run)
		var yawmode = getprop("/autopilot/locks/yaw");
		if ( yawmode != "rudder-hold" ) {
		    # target heading was set earlier in "init" phase
		    setprop("/autopilot/locks/yaw", "rudder-hold");
		}
	    }
	    if ( airspeed > 5 and airspeed < 70 ) {
		# use differential braking for lower speed ground steering
		rudder = getprop("/controls/flight/rudder");
		if ( rudder >= 0 ) {
		    setprop("/controls/gear/brake-right", rudder / 80.0);
		} else {
		    setprop("/controls/gear/brake-left", -rudder / 80.0);
		}
	    } else {
		setprop("/controls/gear/brake-right", 0.0);
		setprop("/controls/gear/brake-left", 0.0);
	    }
	    if ( airspeed > rotate_speed ) {
		# begin to rotate
		var pitch = getprop("/autopilot/settings/target-pitch-deg");
		pitch += dt * 4.0;
		if ( pitch > 10.0 ) {
		    pitch = 10.0;
		}
		setprop("/autopilot/settings/target-pitch-deg", pitch);
	    }
	}

	# adjust target heading to compensate for xtrack error
	var speed_gain = 0;
	if ( airspeed > 0 ) {
	    speed_gain = 1 - (airspeed/100);
	}
	if ( speed_gain < 0.05 ) { speed_gain = 0.05; }
	if ( speed_gain > 1.0 ) { speed_gain = 1.0; }
	var xtrack_comp = xtrack * speed_gain * 1;
	if ( xtrack_comp < -45 ) { xtrack_comp = -45; }
	if ( xtrack_comp > 45 ) { xtrack_comp = 45; }
	var target_heading = rwy_heading - xtrack_comp;
	var current_heading = getprop("/orientation/heading-deg");
	var yaw_error = current_heading - target_heading;
	if ( yaw_error < -180 ) { yaw_error += 360.0; }
	if ( yaw_error > 180 ) { yaw_error -= 360.0; }
	setprop("/autopilot/internal/yaw-error-deg", yaw_error);

	var agl = getprop("/position/altitude-agl-ft");
	var wow = 0;
	wow += getprop("/gear/gear[0]/wow");
	wow += getprop("/gear/gear[1]/wow");
	wow += getprop("/gear/gear[2]/wow");
	if ( agl > 25.0 and wow == 0 ) {
	    # airborne
	    # gear up
	    setprop("/controls/gear/gear-down", 0);
	    setprop("/uas/state", "launch-climbout");
	    print("state -> launch-climbout");
	    # target speed 250 kts
	    setprop("/autopilot/locks/speed", "speed-with-throttle");
	    setprop("/autopilot/settings/target-speed-kt", climbout_speed_kt);
	    # turn off rudder heading hold
	    setprop("/autopilot/locks/yaw", "");
	    setprop("/controls/flight/rudder", 0.0);	    
	}
    } elsif ( state == "launch-takeoff-winch" ) {
	# keep throttle off until we release the winch line
	throttle = 0.0;
	setprop("/controls/engines/engine[0]/throttle", throttle);
	setprop("/controls/engines/engine[1]/throttle", throttle);
	setprop("/controls/engines/engine[2]/throttle", throttle);
	setprop("/controls/engines/engine[3]/throttle", throttle);

	# release parking brake
	setprop("/controls/gear/brake-parking", 0);

	var winch_open = getprop("/sim/hitches/winch/open");
	if ( winch_open == 0 ) {
	    var winch_speed = getprop("/sim/hitches/winch/winch/rel-speed");
	    winch_speed += dt * 0.2;
	    if ( winch_speed > 0.6 ) {
		winch_speed = 0.6;
	    }
	    setprop("/sim/hitches/winch/winch/rel-speed", winch_speed);
	}
	var airspeed = getprop("/velocities/airspeed-kt");

	var groundspeed = getprop("/velocities/groundspeed-kt");
	if ( groundspeed > 35.0 ) {
	    # assume we are pretty much flying by now if our ground
	    # speed is over 40

	    # gear up
	    setprop("/controls/gear/gear-down", 0);
	    setprop("/sim/hitches/winch/open", 1);
	    setprop("/uas/state", "launch-climbout");
	    print("state -> launch-climbout");
	    # target speed 250 kts
	    setprop("/autopilot/locks/speed", "elevator");
	    setprop("/autopilot/settings/target-speed-kt", climbout_speed_kt);
	    # turn off rudder heading hold
	    setprop("/autopilot/locks/yaw", "");
	    setprop("/controls/flight/rudder", 0.0);	    
	}
    } elsif ( state == "launch-climbout" ) {
	var agl = getprop("/position/altitude-agl-ft");
	var alt = getprop("/position/altitude-ft");

	#var current_pitch_target
	#    = getprop("/autopilot/settings/target-pitch-deg");
	#if ( current_pitch_target < 9.9 ) {
	#    current_pitch_target += 8*dt;
	#} elsif ( current_pitch_target > 10.1 ) {
	#    current_pitch_target -= 2*dt;
	#} else {
	#    current_pitch_target = 10;
	#}
	#setprop("/autopilot/locks/altitude", "pitch-hold");
	#setprop("/autopilot/settings/target-pitch-deg", current_pitch_target);

	if ( agl > climbout_alt_ft ) {
	    # flaps up, set target altitude, activate route manager
	    var target_alt = int((alt + operational_alt) / 100) * 100;
	    setprop("/controls/flight/flaps", 0);
	    setprop("/controls/flight/flapscommand", 0);
	    var routesize = getprop("/autopilot/route-manager/route/num");
	    if ( routesize > 0 ) {
		#setprop("/autopilot/route-manager/active", 1);
		setprop("/autopilot/route-manager/current-wp", 0);
	    }
	    setprop("/autopilot/locks/heading", "true-heading-hold");

	    # if next task is to fly the route and we are flying pitch
	    # angle on climbout, otherwise, comment these lines out
	    #setprop("/autopilot/settings/target-pitch-deg", 2);
	    #setprop("/autopilot/settings/target-altitude-ft", target_alt);
	    #setprop("/autopilot/locks/altitude", "altitude-hold");

	    var routesize = getprop("/autopilot/route-manager/route/num");
	    if ( routesize > 0 ) {
		setprop("/uas/state", "route");
		print("state -> route");
		setprop("/uas/camera-target", "Next Waypoint");
	    } else {
		setprop("/uas/state", "circle");
		print("state -> circle");
		if ( mode == "carrier" ) {
		    setprop("/uas/camera-target", "Carrier");
		} else {
		    setprop("/uas/camera-target", "Operator");
		}
	    }
	    update_camera_target();
	}
    } elsif ( state == "refuel" and enable_tanker ) {
	var fuel_probe_horiz = 0; # (m) positive moves the airplane left relative to the tanker
	var fuel_probe_vert = 0; # (ft) postiive moves the airplane up relative to the tanker
	var fuel_probe_trail = 0; # (m)

	#print("tanker hdg = ", tanker_heading, " spd = ", tanker_speed );
	var tanker_bearing = getprop("/ai/models/tanker/radar/bearing-deg");
	var tanker_dist = getprop("/ai/models/tanker/radar/range-nm");
	var tanker_alt = getprop("/ai/models/tanker/position/altitude-ft");
	var tanker_lon = getprop("/ai/models/tanker/position/longitude-deg");
	var tanker_lat = getprop("/ai/models/tanker/position/latitude-deg");

	set_runway_location_by_coord( tanker_lon, tanker_lat,
				      tanker_alt + fuel_probe_vert,
				      tanker_heading );
	calc_xtrack();

	var own_alt = getprop("/position/altitude-ft");

	# override xtrack() calcs
	var course_from = tanker_bearing - 180.0;
	if ( course_from < 0 ) { course_from += 360.0; }
	var deg_offset = course_from - tanker_heading;
	if ( deg_offset < -180 ) { deg_offset += 360.0; }
	runway_distance_to = tanker_dist * NM2M;
	xtrack = math.sin( deg_offset * D2R ) * runway_distance_to;
	rwy_dist = math.cos( deg_offset * D2R ) * runway_distance_to;
	setprop("/uas/xtrack", xtrack);
	setprop("/uas/rwy-dist", rwy_dist);

	if ( refuel_state == 0 ) {
	    # enter holding position
	    fuel_probe_horiz = 100; 
	    fuel_probe_vert = -30;
	    fuel_probe_trail = 200;
	    if ( math.abs(rwy_dist) < (fuel_probe_trail * 1.1) and
		math.abs(own_alt - tanker_alt) < 50 ) {
		refuel_state = 1;
	    }
	} elsif ( refuel_state == 1 ) {
	    # slide over into position
	    fuel_probe_horiz = 1.5;
	    fuel_probe_vert = -9;
	    fuel_probe_trail = 35;
	    if ( math.abs( fuel_probe_horiz + xtrack ) < 1.0 ) {
		timer_start = time;
		f14.refuel_probe_switch_cycle();
		refuel_state = 2;
	    }
	} elsif ( refuel_state == 2 ) {
	    # move to final refueling position
	    fuel_probe_horiz = 1.5;
	    fuel_probe_vert = -9;
	    fuel_probe_trail = 30;
	    if ( time > timer_start + 45 ) {
		timer_start = time;
		refuel_state = 3;
	    }
	} elsif ( refuel_state == 3 ) {
	    # don't waste too much time dropping off, lets just bug
	    # out after 15 seconds to keep life exciting
	    fuel_probe_horiz = 5;
	    fuel_probe_vert = -14;
	    fuel_probe_trail = 100;
	    if ( time > timer_start + 15 ) {
		f14.refuel_probe_switch_cycle();
		f14.refuel_probe_switch_cycle();
		setprop("/uas/state", "gohome");
		print("state -> gohome");
	    }
	}

	var own_indspeed
	    = getprop("/instrumentation/airspeed-indicator/indicated-speed-kt");
	var speed_ratio = 1;
	if ( own_groundspeed > 0 ) {
	    speed_ratio = own_indspeed / own_groundspeed;
	}

	var speed_offset = 0;
	var target_speed = 0;
	if ( rwy_dist < -1000 and rwy_distance_to > 4000 ) {
	    # "well" behind the tanker and further than 4000m away, fly direct to
	    var heading = rwy_course_to - 180;
	    if ( heading < 0 ) { heading += 360.0; }
	    setprop("/autopilot/settings/true-heading-deg", heading);

	    # speed offset based on total distance from
	    speed_offset = (rwy_distance_to - fuel_probe_trail)*M2NM * 200;
	    target_speed = tanker_speed * speed_ratio + speed_offset;
	} else {
	    # compute target heading to fly the runway centerline with
	    # wind compensation
	    var gain = 0.0499 * (550 - own_indspeed) / 300 + 0.0001;
	    if ( gain < 0.0001 ) { gain = 0.0001; }
	    if ( gain > 0.05 ) { gain = 0.05; }
	    var xtrack_comp = (xtrack + fuel_probe_horiz) * gain;
	    if ( xtrack_comp < -45 ) { xtrack_comp = -45; }
	    if ( xtrack_comp > 45 ) { xtrack_comp = 45; }
	    var target_crs = (tanker_heading - xtrack_comp);
	    var truespeed
		= getprop("/instrumentation/airspeed-indicator/true-speed-kt");
	    var windtri = wind_course( target_crs, truespeed, 0 );

	    # account for the fact that we might not be flying the same
	    # way we are pointing.  In real life accurate beta is hard to
	    # come by so maybe at some point I need to think of a better
	    # way (involving ground track heading perhaps) to zero out
	    # this potential bias.
	    var beta = getprop("/orientation/side-slip-deg");
	    windtri.heading += beta;
	    if ( windtri.heading < 0.0 ) { windtri.heading += 360.0; }
	    if ( windtri.heading >= 360.0 ) { windtri.heading -= 360.0; }

	    setprop("/autopilot/settings/true-heading-deg", windtri.heading);

	    # speed offset based on distance along tanker path
	    var dist_offset = -rwy_dist - fuel_probe_trail;
	    speed_offset = dist_offset * M2NM * 200;
	    if ( dist_offset < 0 ) {
		# leading (back off speed more quickly)
		target_speed = tanker_speed * speed_ratio + speed_offset * 20;
	    } else {
		# trailing (need to catch up)
		target_speed = tanker_speed * speed_ratio + speed_offset;
	    }
	}

	if ( target_speed < 200 ) { target_speed = 200; }
	if ( target_speed > max_speed ) { target_speed = max_speed; }
	setprop("/autopilot/settings/target-speed-kt", target_speed);

	manage_speedbrake( target_speed, 0 );

	var alt_diff = tanker_alt - own_alt;
	if ( alt_diff > 2000 ) {
	    var current_pitch_target
		= getprop("/autopilot/settings/target-pitch-deg");
	    if ( current_pitch_target < 14.9 ) {
		current_pitch_target += dt;
	    } elsif ( current_pitch_target > 15.1 ) {
		current_pitch_target -= dt;
	    } else {
		current_pitch_target = 15;
	    }
	    setprop("/autopilot/locks/altitude", "pitch-hold");
	    setprop("/autopilot/settings/target-pitch-deg",
		    current_pitch_target);
	} elsif ( alt_diff > 1000 ) {
	    var pitch = 10 + (alt_diff - 1000) / 200;
	    setprop("/autopilot/locks/altitude", "pitch-hold");
	    setprop("/autopilot/settings/target-pitch-deg", pitch);
	} elsif ( alt_diff > 500 ) {
	    var pitch = 5 + (alt_diff - 500) / 100;
	    setprop("/autopilot/locks/altitude", "pitch-hold");
	    setprop("/autopilot/settings/target-pitch-deg", pitch);
	} else {
	    setprop("/autopilot/locks/altitude", "altitude-hold");
	    setprop("/autopilot/settings/target-altitude-ft",
		    tanker_alt + fuel_probe_vert);
	}
    } elsif ( state == "route" ) {
        var wp_capture_dist_m = 50.0;
	var target_alt = getprop("/uas/flight-altitude-ft");
	setprop("/autopilot/settings/target-altitude-ft", target_alt);
        var target_speed = route_speed;
        setprop("/autopilot/settings/target-speed-kt", target_speed);
	setprop("/autopilot/locks/altitude", "throttle");

	var curwp = getprop("/autopilot/route-manager/current-wp");
	var routesize = getprop("/autopilot/route-manager/route/num");

        # fetch previous waypoint
        var wp_prev = geo.aircraft_position(); # temp init
        if ( curwp > 0 ) {
            var prevwp = curwp - 1;
            var wp_path = "/autopilot/route-manager/route/wp[" ~ prevwp ~ "]";
            var wp_node = props.globals.getNode(wp_path);
            if ( wp_node != nil ) {
	        wp_prev.set_lon( wp_node.getChild("longitude-deg").getValue());
	        wp_prev.set_lat( wp_node.getChild("latitude-deg").getValue());
            }
        }

        # fetch current waypoint
	var wp_cur = geo.aircraft_position(); # temp init
        var wp_path = "/autopilot/route-manager/route/wp[" ~ curwp ~ "]";
        var wp_node = props.globals.getNode(wp_path);
        if ( wp_node != nil ) {
	    wp_cur.set_lon( wp_node.getChild("longitude-deg").getValue());
	    wp_cur.set_lat( wp_node.getChild("latitude-deg").getValue());
        }

        var course_to = 0.0;
        var dist_m = 0.0;

	cur_coord = geo.aircraft_position();

        if ( curwp > 0 ) {
	    var leg_course = wp_prev.course_to( wp_cur );
	    var leg_dist = wp_prev.distance_to( wp_cur );
	    var direct_course = cur_coord.course_to( wp_cur );
	    var direct_dist = cur_coord.distance_to( wp_cur );
            var angle = leg_course - direct_course;
            if ( angle < -180 ) {
	        angle += 360.0;
            } elsif ( angle > 180 ) {
	        angle -= 360.0;
            }
            var xtrack = math.sin( angle * D2R ) * direct_dist;
            dist_m = math.cos( angle * D2R ) * direct_dist;
            setprop("/uas/xtrack", xtrack);
            setprop("/uas/track-dist", dist_m);
	    var xtrack_comp = xtrack * xtrack_route_gain;
	    if ( xtrack_comp < -45 ) { xtrack_comp = -45; }
	    if ( xtrack_comp > 45 ) { xtrack_comp = 45; }
	    course_to = leg_course - xtrack_comp;
            if ( course_to < 0.0 ) {
                course_to += 360.0;
            } elsif ( course_to > 360.0 ) {
                course_to -= 360.0;
            }
        } else {
	    course_to = cur_coord.course_to( wp_cur );
	    dist_m = cur_coord.distance_to( wp_cur );
            setprop("/uas/xtrack", 0.0);
            setprop("/uas/track-dist", dist_m);
        }
	#printf("course_to = %.1f dist_m = %.0f\n", course_to, dist_m);

	# advance to next waypoint if 'close enough'
        if ( dist_m <= wp_capture_dist_m ) {
	    if ( curwp < routesize - 1 ) {
		curwp += 1;
		setprop("/autopilot/route-manager/current-wp", curwp);
	    } else {
	        # route finished, circle last waypoint untilcommanded otherwise.
	        # disable route manager (if active)
	        # setprop("/autopilot/route-manager/active", 0);
	        setprop("/uas/state", "circle");
	        print("state -> circle");
	        setprop("/uas/camera-target", "Operator");
	        update_camera_target();
	    }
	}

	var truespeed =
	    getprop("/instrumentation/airspeed-indicator/true-speed-kt");
	var windtri = wind_course( course_to, truespeed, 0 );
	setprop("/autopilot/settings/true-heading-deg", windtri.heading);
    } elsif ( state == "circle" ) {
	var target_alt = getprop("/uas/flight-altitude-ft");
	setprop("/autopilot/settings/target-altitude-ft", target_alt);

	circle_coord.set_lon( getprop("/sim/input/click/longitude-deg") );
	circle_coord.set_lat( getprop("/sim/input/click/latitude-deg") );
	circle_coord.set_alt( getprop("/sim/input/click/elevation-ft")*FT2M );
	var cur_coord = geo.aircraft_position();
	var course_to = cur_coord.course_to( circle_coord );
	var dist_to = cur_coord.distance_to( circle_coord );
	#printf("course_to = %.1f dist_to = %.0f\n", course_to, dist_to);

	# compute ideal heading if at ideal radius
	var ideal_hdg = course_to + 90;
	if ( ideal_hdg > 360.0 ) { ideal_hdg -= 360.0; }

	# compute idea radius for a "circle_bank" degree bank
	var radius_ft = (circle_speed * circle_speed)
	    / (11.23*math.tan(0.01745 * circle_bank));
	var radius_m = radius_ft * FT2M;

	# adjust target heading for our actual radius
	var target_hdg = ideal_hdg;
	if ( dist_to < radius_m ) {
	    # inside circle, adjust target heading to expand our
	    # circling radius
	    var offset_deg = 90 * (1.0 - dist_to / radius_m);
	    target_hdg += offset_deg;
	    if ( target_hdg > 360.0 ) { target_hdg -= 360.0; }
	} elsif ( dist_to > radius_m ) {
	    # outside circle, adjust target heading to tighten our
	    # circling radius
	    var offset_dist = dist_to - radius_m;
	    if ( offset_dist > radius_m ) { offset_dist = radius_m; }
	    var offset_deg = 90 * offset_dist / radius_m;
	    target_hdg -= offset_deg;
	    if ( target_hdg < 0.0 ) { target_hdg += 360.0; }
	}
	var truespeed =
	    getprop("/instrumentation/airspeed-indicator/true-speed-kt");
	var windtri = wind_course( target_hdg, truespeed, 0 );
	setprop("/autopilot/settings/true-heading-deg", windtri.heading);

	setprop("/autopilot/settings/target-speed-kt", circle_speed);
	manage_speedbrake( circle_speed, 5 );
		
	var airspeed = getprop("/velocities/airspeed-kt");
	if ( airspeed <= flap_speed_kt ) {
	    setprop("/controls/flight/flaps", 1);
	    setprop("/controls/flight/flapscommand", 1);	
	} else if ( airspeed > flap_speed_kt + 10 ) {
	    setprop("/controls/flight/flaps", 0);
	    setprop("/controls/flight/flapscommand", 0);
	}
    } elsif ( state == "heading" ) {
	var target_alt = getprop("/uas/flight-altitude-ft");
	setprop("/autopilot/settings/target-altitude-ft", target_alt);

	# adjust target heading for our actual radius
	var target_hdg = getprop("/uas/wmi/teleop-hdg-deg");

	var truespeed =
	    getprop("/instrumentation/airspeed-indicator/true-speed-kt");
	var windtri = wind_course( target_hdg, truespeed, 0 );
	setprop("/autopilot/settings/true-heading-deg", windtri.heading);

	setprop("/autopilot/settings/target-speed-kt", max_speed);
	manage_speedbrake( max_speed, 5 );
		
	var airspeed = getprop("/velocities/airspeed-kt");
	if ( airspeed <= flap_speed_kt ) {
	    setprop("/controls/flight/flaps", 1);
	    setprop("/controls/flight/flapscommand", 1);	
	} else if ( airspeed > flap_speed_kt + 10 ) {
	    setprop("/controls/flight/flaps", 0);
	    setprop("/controls/flight/flapscommand", 0);
	}
    } elsif ( state == "gohome" ) {
	if ( rwy_distance_to > 2000 ) {
	    # fly direct to
	    var alt_offset = math.sin(glideslope*D2R) * rwy_distance_to;
	    var target_alt = (rwy_coord.alt() + alt_offset) * M2FT;
	    if ( target_alt < rwy_coord.alt()*M2FT + pattern_height_ft ) {
		target_alt = rwy_coord.alt()*M2FT + pattern_height_ft;
	    }
	    setprop("/autopilot/locks/altitude", "throttle");
	    setprop("/autopilot/settings/target-altitude-ft", target_alt);

	    # optimal speed is a forward speed that allows us to
	    # arrive at the destination about the same time that we
	    # decend to the correct altitude.
	    var alt = getprop("/position/altitude-ft");
	    var alt_diff = alt - pattern_height_ft;
	    var secs_to_alt = 60 * alt_diff / 3000; # 3k fpm decent
	    var mps = rwy_distance_to / secs_to_alt;
	    var optimal_kts = mps * MPS2KT;

	    var heading = rwy_course_to - 180;
	    if ( heading < 0 ) { heading += 360.0; }
	    var truespeed
		= getprop("/instrumentation/airspeed-indicator/true-speed-kt");
	    var windtri = wind_course( heading, truespeed, 0 );
	    setprop("/autopilot/locks/heading", "true-heading-hold");
	    setprop("/autopilot/settings/true-heading-deg", windtri.heading);
	    
	    var heading_error
		= getprop("/autopilot/internal/true-heading-error-deg");
	    var target_speed = 0;
	    if ( math.abs(heading_error) > 60 ) {
		# if we are turned in the wrong direction, keep our
		# speed low until we get aimed more towards home.
		target_speed = downwind_speed_kt;
	    } else {
		target_speed = optimal_kts;

		# speed limit based on total distance from rwy so we
		# can arrive in the pattern at pattern speed
		speed_offset = (rwy_distance_to - 2000) / 40;
		speed_limit = downwind_speed_kt + speed_offset;
		if ( target_speed > speed_limit ) {
		    target_speed = speed_limit;
		}
		if ( target_speed < downwind_speed_kt ) {
		    target_speed = downwind_speed_kt;
		}
		if ( target_speed > max_speed ) { target_speed = max_speed; }
	    }
	    setprop("/autopilot/settings/target-speed-kt", target_speed);
	    manage_speedbrake( target_speed, 5 );

	    var airspeed = getprop("/velocities/airspeed-kt");
	    if ( airspeed <= flap_speed_kt ) {
		setprop("/controls/flight/flaps", 1);
		setprop("/controls/flight/flapscommand", 1);	
	    } else if ( airspeed > flap_speed_kt + 10 ) {
		setprop("/controls/flight/flaps", 0);
		setprop("/controls/flight/flapscommand", 0);
	    }
	} else {
	    setprop("/uas/camera-target", "Carrier");
	    setprop("/autopilot/locks/heading", "true-heading-hold");
	    update_camera_target();
	    approach_dw_alt = getprop("/position/altitude-ft");
	    setprop("/uas/state", "downwind");
	    print("state -> downwind");
	}
    } elsif ( state == "downwind" ) {
	# fly downwind to setup for rwy_gate entrance
	# currently hardwired for a "lefthand" approach

	setprop("/autopilot/settings/target-speed-kt", downwind_speed_kt);
	manage_speedbrake( downwind_speed_kt, 5 );

	var cur_alt = getprop("/position/altitude-ft");

	compute_approach_metrics( dt );

	# lock on glideslope when close enough
	if ( approach_alt_error < 0.0 ) {
	    approach_lock_glideslope = 1;
	}
	setprop("/uas/approach/gs-lock", approach_lock_glideslope);

	if ( approach_lock_glideslope ) {
	    # fly decent rate based altitude profile

	    var alt_bias = 25; # fly the downwind leg this much higher
			       # than the computed ideal so we don't
			       # end up too low on the base/final leg.

	    # offset target decent rate to get on the glide slope
	    var offset_rate = ((approach_alt_error + alt_bias)*M2FT / 60) * 5;
	    approach_vertspeed_fps += offset_rate;
	    # cap at +appoach_max_climb_rate/-1000 fpm off from ideal
	    if ( approach_vertspeed_fps > approach_max_climb_rate ) {
		approach_vertspeed_fps = approach_max_climb_rate;
	    }
	    if ( approach_vertspeed_fps < approach_max_decent_rate ) {
		approach_vertspeed_fps = approach_max_decent_rate;
	    }
	} else {
	    # maintain current altitude until glideslope locked, climb
	    # only if below pattern altitude
	    var target_alt = approach_dw_alt;
	    var pattern_alt = rwy_coord.alt()*M2FT + pattern_height_ft;
	    if ( target_alt < pattern_alt ) {
		target_alt = pattern_alt;
	    }
	    var alt_error = target_alt - cur_alt;

	    # offset target decent rate to get on the glide slope
	    var offset_rate = ((alt_error*M2FT) / 60) * 5;
	    approach_vertspeed_fps += offset_rate;
	    # cap at +appoach_max_climb_rate/-1000 fpm off from ideal
	    if ( approach_vertspeed_fps > approach_max_climb_rate ) {
		approach_vertspeed_fps = approach_max_climb_rate;
	    }
	    if ( approach_vertspeed_fps < approach_max_decent_rate ) {
		approach_vertspeed_fps = approach_max_decent_rate;
	    }
	}
	setprop("/uas/approach/target-vert-speed", approach_vertspeed_fps);

	setprop("/autopilot/locks/altitude", "vertical-speed-hold");
	setprop("/autopilot/settings/target-rate-of-climb",
		approach_vertspeed_fps);

	# desired xtrack error for outbound leg is the turning
	# diameter (+ fudge factor).  Note: xtrack is negative for a
	# left handed approach

	# compute target heading to fly the outbound offset runway
	# centerline with wind compensation
	var error = approach_diameter + xtrack;
	var xtrack_comp = error * xtrack_downwind_gain;
	if ( xtrack_comp < -60 ) { xtrack_comp = -60; }
	if ( xtrack_comp > 60 ) { xtrack_comp = 60; }
	var target_crs = (rwy_recip + xtrack_comp);
	var windtri = {};
	var truespeed
	    = getprop("/instrumentation/airspeed-indicator/true-speed-kt");
	windtri = wind_course( target_crs, truespeed, (mode == "carrier") );
	setprop("/autopilot/settings/true-heading-deg", windtri.heading);

	var heading_error
	    = getprop("/autopilot/internal/true-heading-error-deg");
	var airspeed = getprop("/velocities/airspeed-kt");
	if ( airspeed <= flap_speed_kt ) {
	    setprop("/controls/flight/flaps", 1);
	    setprop("/controls/flight/flapscommand", 1);
	}

	# detect ready to turn base (slow enough, close enough to
	# xtrack target, close enough to outbound heading, and past
	# the 45 turn point and not above the ideal glide slope
	# altitude)
	if ( airspeed < downwind_speed_kt + 10
	     and math.abs(error) < (approach_diameter * 0.25)
	     and math.abs(heading_error) < 15
	     and -rwy_dist > approach_45_dist
	     and approach_alt_error > -5 )
	{
	    var target_heading = getprop("/autopilot/settings/true-heading-deg");
	    target_heading -= 90; # 90 degree left turn
	    setprop("/autopilot/settings/true-heading-deg", target_heading);
	    setprop("/autopilot/settings/target-speed-kt", final_speed_kt);
	    setprop("/controls/flight/flaps", 1);
	    setprop("/controls/flight/flapscommand", 1);
	    setprop("/controls/flight/ground-spoilers-armed", 1);
	    setprop("/uas/state", "final");
	    print("state -> final");
	}
    } elsif ( state == "base" ) {
	# fly base leg to setup for rwy_gate entrance
	# currently hardwired for a "lefthand" approach

	# compute angle of remainder of turn to rwy heading
	var current_heading = getprop("/orientation/heading-deg");
	var hdg_diff = rwy_heading - current_heading;
	if ( hdg_diff < -180 ) { hdg_diff += 360.0; }
	if ( hdg_diff > 180 ) { hdg_diff -= 360.0; }
	
	# percent of 180 deg turn
	var turn_frac = math.abs(hdg_diff) / 180.0;

	# compute turning diameter for current speed
	var airspeed = getprop("/velocities/airspeed-kt");
	var radius_ft = (airspeed*airspeed) / (11.23*math.tan(0.01745*max_pattern_bank));
	var radius_m = radius_ft * FT2M;

	compute_approach_metrics( dt );
	
	var alt = ( rwy_coord.alt()
		    + math.tan(glideslope*D2R) * approach_dist_to_touchdown ) * M2FT;
	if ( alt > rwy_coord.alt()*M2FT + pattern_height_ft ) {
	    alt = rwy_coord.alt()*M2FT + pattern_height_ft;
	}
	if ( alt < rwy_gate.alt()*M2FT ) {
	    alt = rwy_gate.alt()*M2FT;
	}
	setprop("/autopilot/settings/target-altitude-ft", alt);

	# detect ready to turn final (close enough to base heading)
	var heading_error = getprop("/autopilot/internal/true-heading-error-deg");
	# within 135 degrees of final rwy heading, start thinking
	# about the turn to final
	if ( turn_frac < ( 6.0 / 8.0 ) ) {
	    setprop("/autopilot/settings/target-speed-kt", final_speed_kt);
	    setprop("/controls/flight/flaps", 1);
	    setprop("/controls/flight/flapscommand", 1);
	    setprop("/controls/flight/ground-spoilers-armed", 1);
	    closing_speed_kt = 0.0;
	    setprop("/uas/state", "final");
	    print("state -> final");
	}
    } elsif ( state == "final" ) {
	# lower gear when airspeed < gear_down_speed_kt
	var airspeed = getprop("/velocities/airspeed-kt");
	if ( airspeed <= gear_down_speed_kt ) {
	    setprop("/controls/gear/gear-down", 1);
	}
	if ( mode == "carrier" ) {
	    if ( -rwy_dist < 4000.0 ) {
		setprop("/controls/gear/tailhook", 1);
		# make sure gear is dropped now if it aint already,
		# cause we'll need it real soon!
		setprop("/controls/gear/gear-down", 1);
	    }
	}

	var heading_error
	    = getprop("/autopilot/internal/true-heading-error-deg");

	var cur_alt = getprop("/position/altitude-ft");

	compute_approach_metrics( dt );

	# lock on glideslope when close enough
	if ( approach_alt_error < 0.0 ) {
	    approach_lock_glideslope = 1;
	}
	setprop("/uas/approach/gs-lock", approach_lock_glideslope);

	if ( approach_lock_glideslope ) {
	    # fly decent rate based altitude profile

	    # offset target decent rate to get on the glide slope
	    var offset_rate = (approach_alt_error*M2FT / 60) * 5;
	    approach_vertspeed_fps += offset_rate * 1.5;
	    # cap at +appoach_max_climb_rate/-1000 fpm off from ideal
	    if ( approach_vertspeed_fps > approach_max_climb_rate ) {
		approach_vertspeed_fps = approach_max_climb_rate;
	    }
	    if ( approach_vertspeed_fps < approach_max_decent_rate ) {
		approach_vertspeed_fps = approach_max_decent_rate;
	    }
	} else {
	    # maintain current altitude until glideslope locked, climb
	    # only if below pattern altitude
	    var target_alt = approach_dw_alt;
	    var pattern_alt = rwy_coord.alt()*M2FT + pattern_height_ft;
	    if ( target_alt < pattern_alt ) {
		target_alt = pattern_alt;
	    }
	    var alt_error = target_alt - cur_alt;

	    # offset target decent rate to get on the glide slope
	    var offset_rate = ((alt_error*M2FT) / 60) * 5;
	    approach_vertspeed_fps += offset_rate;
	    # cap at +appoach_max_climb_rate/-1000 fpm off from ideal
	    if ( approach_vertspeed_fps > approach_max_climb_rate ) {
		approach_vertspeed_fps = approach_max_climb_rate;
	    }
	    if ( approach_vertspeed_fps < approach_max_decent_rate ) {
		approach_vertspeed_fps = approach_max_decent_rate;
	    }
            }
	setprop("/uas/approach/target-vert-speed", approach_vertspeed_fps);

	setprop("/autopilot/locks/altitude", "vertical-speed-hold");
	setprop("/autopilot/settings/target-rate-of-climb",
		approach_vertspeed_fps);

	# compute target heading to fly the runway centerline with
	# wind compensation
	var xtrack_comp = xtrack * xtrack_final_gain;
	if ( xtrack_comp < -45 ) { xtrack_comp = -45; }
	if ( xtrack_comp > 45 ) { xtrack_comp = 45; }
	var target_crs = (rwy_heading - xtrack_comp);
	var windtri = {};
	var truespeed
	    = getprop("/instrumentation/airspeed-indicator/true-speed-kt");
	windtri = wind_course( target_crs, truespeed, (mode == "carrier") );
	#print("true-heading-deg=", windtri.heading);

	# account for the fact that we might not be flying the same
	# way we are pointing.  In real life accurate beta is hard to
	# come by so maybe at some point I need to think of a better
	# way (involving ground track heading perhaps) to zero out
	# this potential bias.
	var beta = getprop("/orientation/side-slip-deg");
	windtri.heading += beta;
	if ( windtri.heading < 0.0 ) { windtri.heading += 360.0; }
	if ( windtri.heading >= 360.0 ) { windtri.heading -= 360.0; }

	setprop("/autopilot/settings/true-heading-deg", windtri.heading);

	var agl = getprop("/position/altitude-agl-ft");
	if ( agl < -approach_vertspeed_fps * flare_gain ) {
	    setprop("/uas/state", "flare");
	    print("state -> flare");
	}
    } elsif ( state == "flare" ) {
	# turn off autothrottle, and cut engines
	setprop("/autopilot/locks/speed", "");

	if ( mode == "runway" ) {
	    # try to level out
	    var agl = getprop("/position/altitude-agl-ft");
	    setprop("/autopilot/settings/target-rate-of-climb",
		    -agl / flare_gain );

	    setprop("/controls/engines/engine[0]/throttle", 0.0);
	    setprop("/controls/engines/engine[1]/throttle", 0.0);
	    setprop("/controls/engines/engine[2]/throttle", 0.0);
	    setprop("/controls/engines/engine[3]/throttle", 0.0);
	}

	var wow = 0;
	wow += getprop("/gear/gear[0]/wow");
	wow += getprop("/gear/gear[1]/wow");
	wow += getprop("/gear/gear[2]/wow");
	if ( wow > 0 ) {
	    setprop("/uas/state", "touchdown");
	    print("state -> touchdown");
	}
    } elsif ( state == "touchdown" ) {
	# apply full brakes
	setprop("/controls/gear/brake-right", 1.0);
	setprop("/controls/gear/brake-left", 1.0);

	if ( mode == "carrier" ) {
	    # go full throttle in case we missed the cable
	    setprop("/controls/engines/engine[0]/throttle", 0.759);
	    setprop("/controls/engines/engine[1]/throttle", 0.759);
	    setprop("/controls/engines/engine[2]/throttle", 0.759);
	    setprop("/controls/engines/engine[3]/throttle", 0.759);

	    var airspeed = getprop("/velocities/airspeed-kt");
	    var wow = 0;
	    wow += getprop("/gear/gear[0]/wow");
	    wow += getprop("/gear/gear[1]/wow");
	    wow += getprop("/gear/gear[2]/wow");
	    if ( wow > 0 and airspeed < 100 ) {
		# caught the wire, shut down the engines
		setprop("/controls/engines/engine[0]/throttle", 0.0);
		setprop("/controls/engines/engine[1]/throttle", 0.0);
		setprop("/controls/engines/engine[2]/throttle", 0.0);
		setprop("/controls/engines/engine[3]/throttle", 0.0);
		setprop("/uas/state", "shutdown");
		print("state -> shutdown");
	    }
	}

	# flaps up to kill lift
	# setprop("/controls/flight/flaps", 0.0);
    } elsif ( state == "shutdown" ) {
	var groundspeed = getprop("/velocities/groundspeed-kt");
	if ( mode == "runway" ) {
	    if ( groundspeed < 10 ) {
		setprop("/controls/flight/ground-spoilers-armed", 0);
		setprop("/uas/state", "end");
		print("state -> end");
	    }
	} elsif ( mode == "carrier" ) {
	    if ( groundspeed < carrier_speed + 10 ) {
		setprop("/controls/flight/ground-spoilers-armed", 0);
		setprop("/uas/state", "end");
		print("state -> end");
	    }
	}
    }
}


var main_loop = func {
    time = getprop("/sim/time/elapsed-sec");
    var dt = time - last_time;
    last_time = time;

    if ( getprop("/uas/master-switch") == 1 ) {
        #print("drone update start:", dt);
	update_state( dt );
        #print("drone update end.");
    }

    settimer(main_loop, 0);
}


setlistener("/sim/signals/fdm-initialized",
            func {
		# master switch starts off
		props.globals.initNode("/uas/master-switch", 0, "BOOL", 1);

		# set some defaults
		setprop("/sim/hud/visibility[1]", 1); # hud on by default
		setprop("/uas/view-mode", "Cockpit");
		setprop("/uas/camera-zoom", "1.0");
                setprop("/uas/emulate-pitch-bobble", 0);

		# initial state
		setprop("/uas/state", "");

		# go
                main_loop();
            });

setlistener("/uas/view-mode", update_view);
