// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Shoulder;

public class SetShoulderPercentSpeed extends Command {
  private Shoulder m_shoulder;
  private double m_percentSpeed;
  private Trigger m_overrideSoftLimitsTrigger;
  private boolean m_softLimitsAreDisabled;
  /** 
   * Creates a new SetShoulderPercentSpeed command.  Sets given percent power/speed until the end,
   * where it sets RPM to zero (that is an attempt to hold arm in place). 
   * @param shoulder - a Shoulder object
   * @param percentSpeed - speed on a scale of [-1,1].
   * I think this is speed in clockwise direction when viewed from robot's left, so negative raises arm.
   */
  public SetShoulderPercentSpeed(Shoulder shoulder, double percentSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shoulder = shoulder;
    m_percentSpeed = percentSpeed;
    m_overrideSoftLimitsTrigger = null;
    m_softLimitsAreDisabled = false;
    addRequirements(m_shoulder);
  }
  /** 
   * Creates a new SetShoulderPercentSpeed command.  Sets given percent power/speed until the end,
   * where it sets RPM to zero (that is an attempt to hold arm in place). 
   * @param shoulder - a Shoulder object
   * @param percentSpeed - speed on a scale of [-1,1].
   * I think this is speed in clockwise direction when viewed from robot's left, so negative raises arm.
   * @param overrideSoftLimitsTrigger - a Trigger (including a JoystickButton) that, while pressed,
   * will allow motion to go beyond the soft position limits.
   */
  public SetShoulderPercentSpeed(Shoulder shoulder, double percentSpeed, Trigger overrideSoftLimitsTrigger){
    this(shoulder, percentSpeed);
    m_overrideSoftLimitsTrigger = overrideSoftLimitsTrigger;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("setting shoulder percent speed to " + m_percentSpeed);
    m_shoulder.getSparkMaxMotor().setPercentSpeed(m_percentSpeed);
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

  void stop(){
    System.out.println("stopping shoulder motor");
    m_shoulder.getSparkMaxMotor().setRPM(0.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
