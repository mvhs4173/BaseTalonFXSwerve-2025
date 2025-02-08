// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.positionCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.ShoulderGoToPosition;
import frc.robot.commands.Wrist2GoToPosition;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Wrist2;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class goToCollectionPositionFromSpeaker extends ParallelCommandGroup {
  Shoulder m_shoulder;
  Wrist2 m_wrist2;
  double timeout;
  /** Creates a new AutoGoToAmpShotPosition. */
  public goToCollectionPositionFromSpeaker(Shoulder shoulder, Wrist2 wrist2) {
m_shoulder = shoulder;
m_wrist2 = wrist2;
timeout = 3.0;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ShoulderGoToPosition(m_shoulder, ShoulderGoToPosition.Method.kRPM, 0.1, 0.0).withTimeout(timeout),
      new Wrist2GoToPosition(m_wrist2, 0.3, 0.0).withTimeout(timeout)
    );
  }
}
