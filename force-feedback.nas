# Variables, need to make controllable
var update_interval = 0.1;


###
# Update different forces for device in <path>
# i.e. /haptic/device[X]/

# First pilot G
var update_pilot_g = func(path) {
  var pilot_path = path.getNode("pilot");
  if(pilot_path == nil) return;  # Bail out if pilot forces are not supported

  # If gain is set to 0, effect is disabled so don't update
  if(pilot_path.getNode("gain").getValue() == 0) return;

  var pilot_x = getprop("/accelerations/pilot/x-accel-fps_sec");  # Sideways, inverted
  var pilot_y = getprop("/accelerations/pilot/y-accel-fps_sec");  # Forwards
  var pilot_z = getprop("/accelerations/pilot/z-accel-fps_sec"); # Take gravity into account (approximation)

  if(pilot_x == nil) pilot_x = 0;
  else pilot_x = -pilot_x / 322.0;

  if(pilot_y == nil) pilot_y = 0;
  else pilot_y = pilot_y / 322.0;

  if(pilot_z == nil) pilot_z = 0;
  else pilot_z = (pilot_z + 32.174) / 322.0;

  # TODO: Axis mapping!
  var axis_x = pilot_path.getNode("x");
  if(axis_x != nil) axis_x.setValue(pilot_y);

  var axis_y = pilot_path.getNode("y");
  if(axis_y != nil) axis_y.setValue(pilot_x);

  var axis_z = pilot_path.getNode("z");
  if(axis_z != nil) axis_z.setValue(pilot_z);
};


# Then control surface forces
var update_surface_forces = func(path) {
  var surface_force_path = path.getNode("surface-force");
  if(surface_force_path == nil) return;  # Bail out if control surface forces are not supported

  # If gain is set to 0, effect is disabled so don't update
  if(surface_force_path.getNode("gain").getValue() == 0) return;

  var surface_force_x = getprop("/fdm/jsbsim/aero/coefficient/ClDa");  # Needs to be inverted
  var surface_force_y = getprop("/fdm/jsbsim/aero/coefficient/Cmde");
  var surface_force_z = getprop("/fdm/jsbsim/aero/coefficient/CnDr");

  if(surface_force_x == nil) surface_force_x = 0;
  else surface_force_x = -surface_force_x / 10000.0;

  if(surface_force_y == nil) surface_force_y = 0;
  else surface_force_y = -surface_force_y / 10000.0;

  if(surface_force_z == nil) surface_force_z = 0;
  else surface_force_z = surface_force_z / 10000.0;

  # TODO: Axis mapping!
  var axis_x = surface_force_path.getNode("x");
  if(axis_x != nil) axis_x.setValue(surface_force_x);

  var axis_y = surface_force_path.getNode("y");
  if(axis_y != nil) axis_y.setValue(surface_force_y);

  var axis_z = surface_force_path.getNode("z");
  if(axis_z != nil) axis_z.setValue(surface_force_z);
};


# Main loop
var update_forces = func {
  # Loop through every device
  var haptic_node = props.globals.getNode("/haptic");

  if(haptic_node != nil)
  {
    var devices = haptic_node.getChildren("device");

    forindex(var i; devices)
    {
      # Call functions for all effects
      update_pilot_g(devices[i]);
      update_surface_forces(devices[i]);
    }
  }

  # Reset timer
  settimer(update_forces, update_interval);
};


###
# Main initialization
_setlistener("/sim/signals/nasal-dir-initialized", func {
  # Set timer for main loop
  settimer(update_forces, update_interval);
});
