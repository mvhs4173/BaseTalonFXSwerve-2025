// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SparkMaxMotorPair extends SubsystemBase {
  private SparkMaxMotor m_motor1;
  private SparkMaxMotor m_motor2;
  private boolean m_reversed;

  /** Creates a new SparkMaxMotorPair. */
  public SparkMaxMotorPair(SparkMaxMotor motor1, SparkMaxMotor motor2, boolean reversed) {
    m_motor1 = motor1;
    m_motor2 = motor2;
    m_reversed = reversed;
  }

  public void setRPM(double RPM){
    if(!m_reversed){
      m_motor1.setRPM(RPM);
      m_motor2.setRPM(RPM);
    } else if (m_reversed){
      m_motor1.setRPM(RPM);
      m_motor2.setRPM(-RPM);
    }
  }

  public double getAveragePosition(){
    return ((m_motor1.getPosition() + m_motor2.getPosition()) / 2);
  }

  public double getMotor1Position(){
    return m_motor1.getPosition();
  }

  public double getMotor2Position(){
    return m_motor2.getPosition();
  }

  public void setPIDCoefficients(double kP, double kI, double kD, double kIZone, double kFeedForward, double kMinOutput, double kMaxOutput){
    m_motor1.setPIDCoefficients(kP, kI, kD, kIZone, kFeedForward, kMinOutput, kMaxOutput);
    m_motor2.setPIDCoefficients(kP, kI, kD, kIZone, kFeedForward, kMinOutput, kMaxOutput);
  }

  public void setPercentSpeed(double percent){
    if (!m_reversed){
      m_motor1.setPercentSpeed(percent);
      m_motor2.setPercentSpeed(percent);
    } else if (m_reversed){
      m_motor1.setPercentSpeed(percent);
      m_motor2.setPercentSpeed(-percent);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
