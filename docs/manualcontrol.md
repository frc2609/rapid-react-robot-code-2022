## When in auto

- DPAD UP: Trim hood up

- DPAD DOWN: Trim hood down

- DPAD LEFT: Trim flywheel RPM down

- DPAD RIGHT: Trim flywheel RPM up

## When in manual

- DPAD UP: Set hood pos up

- DPAD DOWN: Set hood pos down

- DPAD LEFT: Set flywheel RPM down

- DPAD RIGHT: Set flywheel RPM up

- (LEFT/RIGHT) JOYSTICK X AXIS: Manually rotate turret

## Toggle

- autoAimButton

When toggling from auto to manual mode, hood position is stored and set in manual mode (sohood maintains the current state). The rotate should also be maintained since we are not using the encoder for position (just for soft limits)

Last set flywheel speed in manual mode is stored and reapplied when switching back to manual mode. (For example, manual mode RPM at 2000 then switching to auto then back to manual would set the flywheel to 2000 again.)