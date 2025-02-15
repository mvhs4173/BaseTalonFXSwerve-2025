// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TestMotorPair extends SubsystemBase {
  private SparkMaxMotor m_motor1;
  private SparkMaxMotor m_motor2;
  private boolean m_reversed;
  private SparkMaxMotorPair m_MotorPair;

  /** Creates a new TestMotorPair. */
  public TestMotorPair(int canID1, int canID2, boolean reversed) {
    m_motor1 = new SparkMaxMotor(canID1, 1, "motor 1");
    m_motor2 = new SparkMaxMotor(canID2, 1, "motor 2");
    m_MotorPair = new SparkMaxMotorPair(m_motor1, m_motor2, reversed);
  }

  public void setPercentSpeed(double percent){
    m_MotorPair.setPercentSpeed(percent);
    System.out.println("Setting percent speed to " + percent);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
