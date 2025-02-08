// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CollectorRoller extends SubsystemBase {
  private final SparkMaxMotor m_motor = new SparkMaxMotor(Constants.CollectorRollerConstants.kCANId,
    Constants.CollectorRollerConstants.encoderRotationsPerFinalRotation,
    Constants.CollectorRollerConstants.kName);
  /** Creates a new CollectorRoller. */
  public CollectorRoller() {
    m_motor.setToBrakeOnIdle(true);
  }

  public void pullIn(double percentSpeed){
    m_motor.setPercentSpeed(percentSpeed);
  }

  public void pullIn(){
    pullIn(Constants.CollectorRollerConstants.defaultPullInSpeed);
  }

  public void pushOut(double percentSpeed){
    m_motor.setPercentSpeed(-percentSpeed);
  }

  public void pushOut(){
    pushOut(Constants.CollectorRollerConstants.defaultPushOutSpeed);
  }

  public void stop(){
    m_motor.setPercentSpeed(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
