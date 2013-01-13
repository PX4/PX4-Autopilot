setlistener("/sim/model/start-idling", func(idle){
    var run= idle.getBoolValue();
    if(run){
    Startup();
    }else{
    Shutdown();
    }
},0,0);

var Startup = func{

setprop("controls/electric/engine[0]/generator",1);
setprop("controls/electric/engine[1]/generator",1);
setprop("controls/electric/avionics-switch",1);
setprop("controls/electric/battery-switch",1);
setprop("controls/electric/inverter-switch",1);

setprop("controls/lighting/instrument-lights",1);
setprop("controls/lighting/nav-lights",1);
setprop("controls/lighting/beacon",1);
setprop("controls/lighting/strobe",1);

setprop("controls/engines/engine[0]/cutoff",0);
setprop("controls/engines/engine[1]/cutoff",0);
setprop("controls/engines/engine[0]/condition",1);
setprop("controls/engines/engine[1]/condition",1);
setprop("controls/engines/engine[0]/mixture",1);
setprop("controls/engines/engine[1]/mixture",1);
setprop("controls/engines/engine[0]/propeller-pitch",1);
setprop("controls/engines/engine[1]/propeller-pitch",1);

setprop("engines/engine[0]/running",1);
setprop("engines/engine[1]/running",1);

setprop("fdm/jsbsim/propulsion/engine[0]/n2",60);
setprop("fdm/jsbsim/propulsion/engine[1]/n2",60);

setprop("controls/electric/RH-AC-bus",1);
setprop("controls/electric/LH-AC-bus",1);
}

var Shutdown = func{

setprop("controls/electric/engine[0]/generator",0);
setprop("controls/electric/engine[1]/generator",0);
setprop("controls/electric/avionics-switch",0);
setprop("controls/electric/battery-switch",0);
setprop("controls/electric/inverter-switch",0);

setprop("controls/lighting/instrument-lights",0);
setprop("controls/lighting/nav-lights",0);
setprop("controls/lighting/beacon",0);
setprop("controls/lighting/strobe",0);

setprop("controls/engines/engine[0]/cutoff",1);
setprop("controls/engines/engine[1]/cutoff",1);
setprop("controls/engines/engine[0]/condition",0);
setprop("controls/engines/engine[1]/condition",0);
setprop("controls/engines/engine[0]/mixture",0);
setprop("controls/engines/engine[1]/mixture",0);
setprop("controls/engines/engine[0]/propeller-pitch",0);
setprop("controls/engines/engine[1]/propeller-pitch",0);

setprop("engines/engine[0]/running",0);
setprop("engines/engine[1]/running",0);

setprop("fdm/jsbsim/propulsion/engine[0]/n2",0);
setprop("fdm/jsbsim/propulsion/engine[1]/n2",0);

setprop("controls/electric/RH-AC-bus",0);
setprop("controls/electric/LH-AC-bus",0);
}

