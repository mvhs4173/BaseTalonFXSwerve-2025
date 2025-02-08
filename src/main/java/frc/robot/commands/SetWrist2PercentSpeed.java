// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Wrist2;

public class SetWrist2PercentSpeed extends Command {
  private Wrist2 m_wrist2;
  private double m_percentSpeed;
  private Trigger m_overrideSoftLimitsTrigger;
  private boolean m_softLimitsAreDisabled;
  
  /**
   * Creates a new SetWristPercentSpeed command
   * @param wrist2 - a wrist2 object
   * @param percentSpeed - a number in [-1,1] to control speed
   */
  public SetWrist2PercentSpeed(Wrist2 wrist2, double percentSpeed) {
    m_wrist2 = wrist2;
    m_percentSpeed = percentSpeed;
    m_overrideSoftLimitsTrigger = null;
    m_softLimitsAreDisabled = false;
    addRequirements(m_wrist2);
  }
    /**
   * Creates a new SetWristPercentSpeed command
   * @param wrist - a wrist object
   * @param percentSpeed - a number in [-1,1] to control speed
   * @param overrideSoftLimitsTrigger
   */
  public SetWrist2PercentSpeed(Wrist2 wrist2, double percentSpeed, Trigger overrideSoftLimitsTrigger){
    this(wrist2, percentSpeed);
    m_overrideSoftLimitsTrigger = overrideSoftLimitsTrigger;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_wrist2.setPercentSpeed(m_percentSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean disableSoftLimits = m_overrideSoftLimitsTrigger != null && m_overrideSoftLimitsTrigger.getAsBoolean();
    if (disableSoftLimits && !m_softLimitsAreDisabled){
      m_softLimitsAreDisabled = true;
      m_wrist2.getSparkMaxMotor().disableSoftLimits();
    } else if (!disableSoftLimits && m_softLimitsAreDisabled){
      m_wrist2.getSparkMaxMotor().enableSoftLimits();
      m_softLimitsAreDisabled = false;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_wrist2.setPercentSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
