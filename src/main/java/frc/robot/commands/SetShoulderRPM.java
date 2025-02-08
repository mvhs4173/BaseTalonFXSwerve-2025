// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Shoulder;

public class SetShoulderRPM extends Command {
  private Shoulder m_shoulder;
  private double m_RPM;
  Trigger m_overrideSoftLimitsTrigger;
  boolean m_softLimitsAreDisabled;
  /** Creates a new SetShoulderSpeed. */
  /**
   * Creates a new SetShoulderSpeed.
   * @param shoulder - a shoulder object
   * @param RPM - desired revolutions per minute, clockwise when seen from robot's left.  (Up is negative.) 
   */
  public SetShoulderRPM(Shoulder shoulder, double RPM) {
    m_shoulder = shoulder;
    m_RPM = RPM;
    m_overrideSoftLimitsTrigger = null;
    m_softLimitsAreDisabled = false;
    addRequirements(m_shoulder);
  }
  /**
   * Creates a new SetShoulderSpeed that allows you to suspend the soft limits on position
   * @param shoulder - a shoulder object
   * @param RPM - desired revolutions per minute, clockwise when seen from robot's left.  (Up is negative.)
   * @param overrideSoftLimitsTrigger - a Trigger (including a JoystickButton) that, while pressed,
   * will allow motion to go beyond the soft position limits.
   */
  public SetShoulderRPM(Shoulder shoulder, double RPM, Trigger overrideSoftLimitsTrigger){
    this(shoulder, RPM);
    m_overrideSoftLimitsTrigger = overrideSoftLimitsTrigger;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("setting shoulder speed to " + m_RPM + " RPM");
    m_shoulder.getSparkMaxMotor().setRPM(m_RPM);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean disableSoftLimits = m_overrideSoftLimitsTrigger != null && m_overrideSoftLimitsTrigger.getAsBoolean();
    if (disableSoftLimits && !m_softLimitsAreDisabled){
      m_softLimitsAreDisabled = true;
      m_shoulder.getSparkMaxMotor().disableSoftLimits();
    } else if (!disableSoftLimits && m_softLimitsAreDisabled){
      m_shoulder.getSparkMaxMotor().enableSoftLimits();
      m_softLimitsAreDisabled = false;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("stopping shoulder motor");
    m_shoulder.getSparkMaxMotor().enableSoftLimits();
    m_shoulder.getSparkMaxMotor().setRPM(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
