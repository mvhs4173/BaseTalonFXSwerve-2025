// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
/**
 * Command to take a shot, either at given RPM or at full power.
 * This command will never finish: you need to interrupt it when binding
 * it to a trigger.
 */
public class Shoot extends Command {
  private Shooter m_shooter;
  private double m_RPM;
  private boolean m_fullBlast = false;
  /** 
   * Rotate shooter motors ourward at given RMP
   * @param shooter - the Shooter object to control
   * @param RPM - the desired revolutions per minute outward of the shooter wheels
   */
  public Shoot(Shooter shooter, double RPM) {
    m_shooter = shooter;
    m_RPM = -RPM;
    m_fullBlast = false;
    addRequirements(m_shooter);
  }
  /**
   * Rotate shooter wheels outward at maximum power
   * @param shooter - the Shooter object to control
   */
  public Shoot(Shooter shooter) {
    m_shooter = shooter;
    m_fullBlast = true;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_fullBlast){
      m_shooter.setPercentSpeed(-1.0);
    } else {
      m_shooter.setRPM(m_RPM);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.setPercentSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
