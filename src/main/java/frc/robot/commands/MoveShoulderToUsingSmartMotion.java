// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shoulder;

public class MoveShoulderToUsingSmartMotion extends Command {
  private Shoulder m_shoulder;
  private double m_desiredPosition;
  private double m_maxSpeed;
  private double m_maxAcceleration;
  private double m_allowedClosedLoopError;
  
  /**
   * Move should to desired position
   * @param shoulder - the shoulder object
   * @param desiredPosition - destination position in rotations
   * @param maxSpeed - maximum travel speed in rotations per minute
   * @param maxAcceleration = maximum acceleration in rotations per minute per minute
   * @param allowedCloseLoopError - ?  (zero seems to be ok)
   */

  public MoveShoulderToUsingSmartMotion(Shoulder shoulder, double desiredPosition, double maxSpeed, double maxAcceleration, double allowedClosedLoopError) {
    m_shoulder = shoulder;
    m_desiredPosition = desiredPosition;
    m_maxSpeed = maxSpeed;
    m_maxAcceleration = maxAcceleration;
    m_allowedClosedLoopError = allowedClosedLoopError;
    addRequirements(m_shoulder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shoulder.getSparkMaxMotor().doSmartMotion(m_desiredPosition, m_maxSpeed, m_desiredPosition, m_maxAcceleration, m_allowedClosedLoopError);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
