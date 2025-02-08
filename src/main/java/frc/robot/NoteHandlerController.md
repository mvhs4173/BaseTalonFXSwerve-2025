# Note Handler's Xbox Controller Button Assignments

Note handler's Xbox controller must be in position 1 on Driver Station's USB tab.

Button assignments are not done yet.
Rotation directions, clockwise or counter-clockwise, are as seen from the robot’s left.

- **Left joystick** - forward rotates main arm (at shoulder joint) counter-clockwise at rather slow fixed speed, raising it; backward rotates it clockwise, lowering it.  Side-to-side does nothing.  This is treated as an on/off trigger, motion past 50% is considered on.
- **Right joystick** - forward rotates shooter (at wrist joint) clockwise at rather slow fixed speed, raising it if arm is down; backward rotates shooter counter-clockwise.  Side-to-side does nothing.  On/off trigger, no speed control.  On depressing the right joystick, the robot will attempt to hold the shooter at the current angle (relative to the main arm).
- **Back button** (small button to left of central Xbox symbol).  Usually the shoulder and wrist joints are constrained not to push beyond the 'soft limits'.  While the back button is pressed the motions commanded by the joysticks will not be constrained.  This is meant to be used when the robot is started in a non-resting condition: it allows you to manipulate the arm segments into resting condition and then press the start button to reset the zero points on their encoders. 
- **Start button** (small button to right of central Xbox symbol) - set current shoulder and wrist orientations as the zero points. Only do this when the robot is in starting position.
- **A button** - a momentary press starts intake motors, which will run for 5 seconds
- **B button** - while pressed this runs the intake motor backwards, to eject a note
- **Left bumper** - move to position for shot to speaker from position very close to front of speaker (arm down to 0 degrees, shooter c. -30).  You must hold down this button until the shot is taken: the arm will move to collection position when you release it.
- **Right bumper** - shoot for speaker, at full power.  You must hold button down until shot is done.
- **Right & left triggers** act like the bumpers but for the amplifier shots.  Both are treated as on/off switches.

The Y button and the POV (in lower left, with 8 positions) are used when climbing.
- **Y button** - while held down, this raises the arm to the vertical position, ready to engage the hooks with the chain.
- **Top of POV (North position)** - engages the ratchet in the shoulder so arm cannot go backwards (until human resets it)
- **SW, S, and SE POV positions** - from left to right these give increasing power to the shoulder for climbing.  Low power is good for initially engaging the chain (remember, the arm cannot go up if you go too low), middle should be good for climbing with second hook from the top, and high power should work when topmost hook is engaged.  Be very careful if engaged with the topmost hook - the chain may tear off the radio or camera.

Other buttons are not assigned actions now.
