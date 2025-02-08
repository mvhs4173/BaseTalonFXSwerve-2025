// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CollectorRoller;
import frc.robot.subsystems.BeamBreakSensor;
import frc.robot.subsystems.Shooter;

public class IntakeUntilBeamBreak extends Command {
  private CollectorRoller m_CollectorRoller;
  private BeamBreakSensor m_BeamBreakSensor;
  private Shooter m_Shooter;
  //private double m_pullInSpeed;
  private double m_shooterRPM;

  /** Creates a new IntakeUntilBeamBreak. */
  public IntakeUntilBeamBreak(CollectorRoller collectorRoller, BeamBreakSensor beamBreakSensor, Shooter shooter, double shooterRPM) {
    m_CollectorRoller = collectorRoller;
    m_BeamBreakSensor = beamBreakSensor;
    m_Shooter = shooter;
    m_shooterRPM = shooterRPM;
    addRequirements(m_CollectorRoller, m_Shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_CollectorRoller.pullIn();
    m_Shooter.setRPM(m_shooterRPM);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_CollectorRoller.stop();
    m_Shooter.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_BeamBreakSensor.noteIsInShooter();
  }
}
