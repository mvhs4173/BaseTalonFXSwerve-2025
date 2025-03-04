// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

public class AimAtAprilTag extends Command {
   Vision m_Vision;
   Swerve m_Swerve;
   double m_TargetYawDegrees;
   double m_AllowableErrorDegrees;
   boolean m_IsTargetVisible;
   double m_CurrentYawDegrees;
   boolean m_WasTargetVisible;
   double m_kP;
   int m_id;
  /** Creates a new AimAtAprilTag. */
  public AimAtAprilTag(Vision vision, Swerve swerve, double targetYawDegrees, double allowableErrorDegrees, int id) {
    m_Vision = vision;
    m_Swerve = swerve;
    m_TargetYawDegrees = targetYawDegrees;
    m_AllowableErrorDegrees = allowableErrorDegrees;
    m_IsTargetVisible = false;
    m_WasTargetVisible = false; 
    m_kP = 0.1;
    m_id = id;
    SmartDashboard.putNumber("Aim kP", m_kP);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(vision, swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    updateYaw();
  }

  private void updateYaw() {

    PhotonTrackedTarget target = m_Vision.getAprilTag(m_id);
    m_IsTargetVisible = target != null;
    if (m_IsTargetVisible) {
      m_CurrentYawDegrees = m_Vision.getYaw(target);
    } else {
      m_CurrentYawDegrees = 0;
    }
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_WasTargetVisible = m_IsTargetVisible;
    updateYaw();
    if (m_IsTargetVisible) {
      double errorDegreesCW = (m_CurrentYawDegrees - m_TargetYawDegrees);
      double errorRadiansCCW = -errorDegreesCW * 2.0 * Math.PI / 360.0;
      //If positive, rotate clockwise. If negative, rotate counterclockwise
      m_kP = SmartDashboard.getNumber("Aim kP", m_kP);
      double rotationSpeedRadiansPerSecondCCW = errorRadiansCCW * m_kP;
      SmartDashboard.putNumber("rotation speed radians per second CCW", rotationSpeedRadiansPerSecondCCW);
      m_Swerve.drive(new Translation2d(0.0, 0.0),  rotationSpeedRadiansPerSecondCCW, false, false);
    } else {
      double searchSpeedRadiansPerSecondCCW = 4.0;
      m_Swerve.drive(new Translation2d(0.0, 0.0), searchSpeedRadiansPerSecondCCW, false, false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Swerve.drive(new Translation2d(0.0, 0.0), 0.0, false, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    updateYaw();
    if (m_IsTargetVisible) {
      double error = (m_CurrentYawDegrees - m_TargetYawDegrees);
      return Math.abs(error) < m_AllowableErrorDegrees;
    } else {
      return false;
    }
  }
}
