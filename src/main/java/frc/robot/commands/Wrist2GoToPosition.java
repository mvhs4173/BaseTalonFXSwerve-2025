// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Wrist2;
/**
 * Move wrist to desired position (rotations counterclockwise from zero,
 * when looking at robot from its right).
 */
public class Wrist2GoToPosition extends Command {
  private Wrist2 m_wrist2;
  private double m_percentSpeed;
  private double m_absoluteValuePercentSpeed;
  private double m_desiredPosition;
  /** Creates a new WristGoToPosition. */
  public Wrist2GoToPosition(Wrist2 wrist2, double absoluteValuePercentSpeed, double desiredPosition) {
    if (absoluteValuePercentSpeed < 0.0) {
      throw new Error("Wrist2's absolutteValuePercentSpeed must be nonnegative, not " + absoluteValuePercentSpeed);
    }
    m_wrist2 = wrist2;
    m_desiredPosition = desiredPosition;
    m_absoluteValuePercentSpeed = absoluteValuePercentSpeed;
    addRequirements(m_wrist2);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_percentSpeed = m_desiredPosition > m_wrist2.getPosition() ? m_absoluteValuePercentSpeed : -m_absoluteValuePercentSpeed;
    m_wrist2.setPercentSpeed(m_percentSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_wrist2.setPercentSpeed(0.0);
    m_wrist2.holdPosition();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean finished = m_percentSpeed >= 0
      ? m_wrist2.getPosition() >= m_desiredPosition
      : m_wrist2.getPosition() <= m_desiredPosition;
    return finished;
  }
}
