// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
/**
 * Two motor shooter for Notes.
 * Motors should run at same speed, in opposite directions
 */
public class Shooter extends SubsystemBase {
  private final SparkMaxMotor m_leftMotor;
  private final SparkMaxMotor m_rightMotor;
  private double m_rpm;
  private double m_percentSpeed;

  /** Creates a new [old style] Shooter. */
  public Shooter() {
    m_leftMotor = new SparkMaxMotor(Constants.ShooterConstants.Left.kCANId,
      Constants.ShooterConstants.encoderRotationsPerFinalRotation,
      Constants.ShooterConstants.Left.kName);
    m_rightMotor = new SparkMaxMotor(Constants.ShooterConstants.Right.kCANId,
       Constants.ShooterConstants.encoderRotationsPerFinalRotation,
      Constants.ShooterConstants.Right.kName);
    m_leftMotor.setPIDCoefficients(Constants.ShooterConstants.PID.kP,
                                    Constants.ShooterConstants.PID.kI, 
                                    Constants.ShooterConstants.PID.kD,
                                    Constants.ShooterConstants.PID.kIZone, 
                                    Constants.ShooterConstants.PID.kFeedForward,
                                    Constants.ShooterConstants.PID.kMinOutput,
                                    Constants.ShooterConstants.PID.kMaxOutput);
    m_rightMotor.setPIDCoefficients(Constants.ShooterConstants.PID.kP,
                                    Constants.ShooterConstants.PID.kI, 
                                    Constants.ShooterConstants.PID.kD,
                                    Constants.ShooterConstants.PID.kIZone, 
                                    Constants.ShooterConstants.PID.kFeedForward,
                                    Constants.ShooterConstants.PID.kMinOutput,
                                    Constants.ShooterConstants.PID.kMaxOutput);

    setRPM(0.0);
  }

  /**
   * @param rpm: desired speed shooter wheels, in revolutions per minute outward.
   */
  // TODO: is positive rpm clockwise or counterclockwise, looking from motor to shaft or from shaft to motor
  public void setRPM(double rpm){
    m_rpm = rpm;
    m_leftMotor.setRPM(m_rpm);
    m_rightMotor.setRPM(-m_rpm);
  }

  public void setPercentSpeed(double percentSpeed){
    m_percentSpeed = percentSpeed;
    m_leftMotor.setPercentSpeed(+m_percentSpeed);
    m_rightMotor.setPercentSpeed(-m_percentSpeed);
  }

  @Override
  public void periodic() {}

  public void stop(){
    setPercentSpeed(0.0);
  }

  public double getLeftVelocity(){
    return m_leftMotor.getVelocity();
  }
  public double getRightVelocity(){
    return m_rightMotor.getVelocity();
  }
}
