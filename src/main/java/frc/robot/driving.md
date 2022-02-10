# Driving The Robot

## Setup
Make sure the arm is moved to the lowest position

Next, enable the robot and move the hooks so that the INNER hooks is on the far end of the arm. Disable and re-enable the robot again (this resets the hook encoder).

Or, instead of disabling/re-enabling the robot you can just press the Back button while the robot is enabled and the hooks are in the correct position.

## Controls

Left Joystick X Axis (`joystick.getRawAxis(0)`):

- Turns robot left and right

Left Joystick Y Axis (`joystick.getRawAxis(1)`):

- Moves robot forward and backward

Right Joystick X Axis (`joystick.getRawAxis(4)`);

- Moves hooks up and down

Start Button (three horizontal lines):

- Emergency eject balls (if colour sensor fails)

Back Button (two overlapping squares):

- Reset hook encoder (move inner hooks to far end of arm first)

X Button:

- (HOLD) Raise arm into position

Y Button:

- Reset RoboRIO NavX Y-Axis control