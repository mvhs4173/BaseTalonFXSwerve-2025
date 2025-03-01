// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CANId;

public class Elevator extends SubsystemBase {
  /*Need to know: center stage position, where zero is */
  private final SparkMaxMotor m_leftMotor;
  private final SparkMaxMotor m_rightMotor;
  private final SparkMaxMotorPair m_sparkMaxMotorPair;
  private double m_centerStagePositionInches;
  private double m_homePosition;
  private final double m_INITIALPOSITION;
  private double m_desiredPosition;
  private double m_tolerance = 0.5; //TODO: adjust these
  private double m_upwardPercentSpeed = 0.3; //TODO: adjust these
  private double m_downwardPercentSpeed = -0.1; //TODO: adjust these
  private final double m_SAFETOEXTENDPOSITION;
  private final double m_COLLECTIONPOSITION = -16.0; //TODO: adjust these
  private final double m_L1POSITION = 3.0; //TODO: adjust these
  private final double m_L2POSITION = 22.0; //TODO: adjust these
  private final double m_L3POSITION = 37.0; //TODO: adjust these
  private final double m_L4POSITION = 59.0; //TODO: adjust these
  private final double m_UPPERHEIGHTLIMIT = 60.0; //TODO: adjust these
  private final double m_LOWERHEIGHTLIMIT = -16.5; //TODO: adjust these
  private final double m_DISTANCETOLOWERTOSCORE = 10.0; //TODO: adjust these

  /** Creates a new Elevator. */
  public Elevator(CANId leftCanId, CANId rightCanId) {
    m_leftMotor = new SparkMaxMotor(leftCanId, 5, "Left Elevator Motor");
    m_rightMotor = new SparkMaxMotor(rightCanId, 5, "Right Elevator Motor");
    m_leftMotor.setInvert(false);
    m_sparkMaxMotorPair = new SparkMaxMotorPair(m_leftMotor, m_rightMotor, true);
    m_INITIALPOSITION = (m_centerStagePositionInches);
    m_homePosition = (m_INITIALPOSITION + 5);
    m_desiredPosition = m_INITIALPOSITION;
    m_SAFETOEXTENDPOSITION = m_homePosition;
    m_leftMotor.setToBrakeOnIdle(true);
    m_rightMotor.setToBrakeOnIdle(true);
    setDefaultCommand(goToDesiredPosition());
  }

  /*Rotations to inches for elevator motors */
  private double rotationsToInches(double rotations){
    return(rotations //rotations
    * 36             //36 teeth per rotation
    * 5              //5mm per tooth
    / 25.4           //conversion from mm to inches
    );
  }

  private double getCenterStagePositionInches(){
    return (rotationsToInches(m_leftMotor.getPosition()));
  }

  private double getDistanceToDesiredPositionInches(){
    return (m_desiredPosition - m_centerStagePositionInches);
  }

  public boolean isCloseToDesiredPosition(){
    return (Math.abs(m_centerStagePositionInches - m_desiredPosition) < m_tolerance);
  }

  private void goToDesiredPositionInches(){
    if(m_desiredPosition < m_LOWERHEIGHTLIMIT){
      m_desiredPosition = m_LOWERHEIGHTLIMIT;
    } else if (m_desiredPosition > m_UPPERHEIGHTLIMIT){
      m_desiredPosition = m_UPPERHEIGHTLIMIT;
    }
    if(Math.abs(m_desiredPosition - m_centerStagePositionInches) < m_tolerance){
      m_sparkMaxMotorPair.setPercentSpeed(0.0);
    } else if (m_desiredPosition > m_centerStagePositionInches){
      m_sparkMaxMotorPair.setPercentSpeed(m_upwardPercentSpeed);
    } else if (m_desiredPosition < m_centerStagePositionInches){
      m_sparkMaxMotorPair.setPercentSpeed(m_downwardPercentSpeed);
    }
  }

  public Command goToDesiredPosition(){
    return run(() -> goToDesiredPositionInches());
  }

  public Command goToLowerPosition(){
    return runOnce(() -> {m_desiredPosition -= m_DISTANCETOLOWERTOSCORE;});
  }

  public Command goToL4Position(){
    return runOnce(() -> {m_desiredPosition = m_L4POSITION;});
  }

  public Command goToL3Position(){
    return runOnce(() -> {m_desiredPosition = m_L3POSITION;});
  }

  public Command goToL2Position(){
    return runOnce(() -> {m_desiredPosition = m_L2POSITION;});
  }

  public Command goToL1Position(){
    return runOnce(() -> {m_desiredPosition = m_L1POSITION;});
  }

  public Command goToCollectionPosition(){
    return runOnce(() -> {m_desiredPosition = m_COLLECTIONPOSITION;});
  }

  public Command goToSafeToExtendPosition(){
    return runOnce(() -> {m_desiredPosition = m_SAFETOEXTENDPOSITION;});
  }

  public Command goToHomePosition(){
    return runOnce(() -> {m_desiredPosition = m_homePosition;});
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_centerStagePositionInches = getCenterStagePositionInches();
    SmartDashboard.putNumber("Center stage position inches", m_centerStagePositionInches);
    SmartDashboard.putBoolean("Is close to desired position", isCloseToDesiredPosition());
    SmartDashboard.putNumber("Desired position", m_desiredPosition);
    SmartDashboard.putNumber("Distance to desired position inches", getDistanceToDesiredPositionInches());
  }
}
