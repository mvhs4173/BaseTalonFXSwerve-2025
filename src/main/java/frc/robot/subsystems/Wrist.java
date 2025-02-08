// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;

public class Wrist extends SubsystemBase {
  private SparkMaxMotor m_motor;
  private double m_percentSpeed = 0.0;
  private boolean m_isHoldingPosition;
  private double m_positionToHold;
  private double m_previousPosition = 0.0;

  /** Creates a new Wrist. */
  public Wrist() {
    System.out.println("Creating Wrist object");
    throw new Error("You must use wrist2 instead of wrist, the Bosch brushed seat motor with external encoder and worm gear has been replaced by a NEO with a big gear ratio");
  /*
    m_motor = new SparkMaxMotor(
      Constants.WristConstants.kCANId,
      Constants.WristConstants.encoderRotationsPerFinalRotation,
      Constants.WristConstants.kName,
      Constants.WristConstants.kMotorType,
      Constants.WristConstants.encoderType,
      Constants.WristConstants.encoderCountsPerRevolution);
    m_motor.setToBrakeOnIdle(true);
    m_isHoldingPosition = false;
    m_motor.setAndEnableLowerSoftLimit(Constants.WristConstants.lowerSoftLimit); // 0.0
    m_motor.setAndEnableUpperSoftLimit(Constants.WristConstants.upperSoftLimit); // 0.28 or so
  */  
  }

  /**
   * 
   * @return rotational position of shooter in rotations counterclockwise
   */
  public double getPosition(){
    return m_motor.getPosition();
  }

  public void setPercentSpeed(double percentSpeed){
    m_isHoldingPosition = false;
    m_percentSpeed = percentSpeed;
  }

  public void holdPosition(){
    holdPosition(getPosition());
  }

  public void holdPosition(double positionToHold){
    System.out.println("Wrist starting hold mode");
    m_isHoldingPosition = true;
    m_positionToHold = positionToHold; 
  }

  public void setCurrentPositionAsZeroEncoderPosition(){
    m_motor.setCurrentPositionAsZeroEncoderPosition();
  }

  public SparkMaxMotor getSparkMaxMotor(){
    return m_motor;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (m_isHoldingPosition) {
      // bang bang control.  The speeds should depend on the angle from vertical, but we don't know that now
      double positionError = m_positionToHold - getPosition();
      if (positionError > Units.degreesToRotations(2.0)) {
        m_percentSpeed = 0.40;
      } else if (positionError < -Units.degreesToRotations(2.0)){
        m_percentSpeed = -0.05;
      } else {
        m_percentSpeed = 0.0;
      }
    }
    m_motor.setPercentSpeed(m_percentSpeed);
    //send a message if the current position is a very large distance from the previous position
    double currentPosition = getPosition();
    if (Math.abs(currentPosition - m_previousPosition) > 0.1){
      System.out.println("Wrist Position Jumped from" + m_previousPosition + " to " + currentPosition);
    }
    m_previousPosition = currentPosition;
 }
}
