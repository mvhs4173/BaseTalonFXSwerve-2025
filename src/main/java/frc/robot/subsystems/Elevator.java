// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CANId;

public class Elevator extends SubsystemBase {
  /*Need to know: center stage position, where zero is */
  private final SparkMaxMotor m_leftMotor;
  private final SparkMaxMotor m_rightMotor;
  private final SparkMaxMotorPair m_sparkMaxMotorPair;

  private double m_centerStagePositionInches;
  private final double m_INITIALPOSITION = 16;
  private final double m_homePosition = (m_INITIALPOSITION + 5);
  private double m_desiredPosition;
  private final double m_tolerance = 0.1; //TODO: adjust these
  private double m_upwardPercentSpeed = 0.17; //TODO: adjust these 
  private double m_downwardPercentSpeed = -0.12; //TODO: adjust these
  private double m_desiredPercentSpeed = 0.0;
  private final double m_SAFETOEXTENDPOSITION = (m_homePosition + 2);
  private final double m_ELEVATOR_POSITION_OFFSET = 16;
  private final double m_COLLECTIONPOSITION = -19 + m_ELEVATOR_POSITION_OFFSET; //TODO: adjust these
  private final double m_L1POSITION = 3.0 + m_ELEVATOR_POSITION_OFFSET; //TODO: adjust these
  private final double m_L2POSITION = 22.0 + m_ELEVATOR_POSITION_OFFSET; //TODO: adjust these
  private final double m_L3POSITION = 37.0 + m_ELEVATOR_POSITION_OFFSET; //TODO: adjust these
  private final double m_L4POSITION = 59.0 + m_ELEVATOR_POSITION_OFFSET; //TODO: adjust these
  private final double m_UPPERHEIGHTLIMIT = 60.0 + m_ELEVATOR_POSITION_OFFSET; //TODO: adjust these
  private final double m_LOWERHEIGHTLIMIT = -16.5 + m_ELEVATOR_POSITION_OFFSET; //TODO: adjust these
  private final double m_DISTANCETOLOWERTOSCORE = 10.0; //TODO: adjust these
  private double m_distanceToDesiredPosition;


  /** Creates a new Elevator. */
  public Elevator(CANId leftCanId, CANId rightCanId) {
    m_leftMotor = new SparkMaxMotor(leftCanId, (9), "Left Elevator Motor");
    m_rightMotor = new SparkMaxMotor(rightCanId, (9), "Right Elevator Motor");
    m_leftMotor.setInvert(false);
    m_sparkMaxMotorPair = new SparkMaxMotorPair(m_leftMotor, m_rightMotor, true);
    m_desiredPosition = m_INITIALPOSITION;
    m_leftMotor.setToBrakeOnIdle(true);
    m_rightMotor.setToBrakeOnIdle(true);
    //setDefaultCommand(goToDesiredPosition());
    SmartDashboard.putNumber("Elevator percent speed ", 0);
    SmartDashboard.putNumber("Elevator p value ", 0);
    SmartDashboard.putString("IntendedElevatorDirection", "stay");
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
    return (rotationsToInches(m_leftMotor.getPosition()) + m_ELEVATOR_POSITION_OFFSET);
  }

  private double getDistanceToDesiredPositionInches(){
    return (Math.abs(m_desiredPosition - m_centerStagePositionInches));
  }

  public boolean isCloseToDesiredPosition(){
    return (Math.abs(m_centerStagePositionInches - m_desiredPosition) < m_tolerance);
  }

  private void goToDesiredPositionInches(){
    m_desiredPosition = MathUtil.clamp(m_desiredPosition, m_LOWERHEIGHTLIMIT, m_UPPERHEIGHTLIMIT);
    double distanceAllowedFullSpeed = 1; //the distance from the desired position that it is allowed to go full speed
    m_distanceToDesiredPosition = Math.abs(m_distanceToDesiredPosition);
    double p = m_distanceToDesiredPosition / distanceAllowedFullSpeed;
    p = Math.abs(p);
    p = MathUtil.clamp(p, 0.0, 1.0);
    SmartDashboard.putNumber("Elevator p value ", p);
    if(m_distanceToDesiredPosition < m_tolerance){ //m_distanceToDesiredPosition is computed in periodic
      m_desiredPercentSpeed = 0.0;
      SmartDashboard.putString("IntendedElevatorDirection", "stay");
    } else if (m_desiredPosition > m_centerStagePositionInches){
      m_desiredPercentSpeed = p * m_upwardPercentSpeed;
      SmartDashboard.putString("IntendedElevatorDirection", "up");
    } else if (m_desiredPosition < m_centerStagePositionInches){
      SmartDashboard.putString("IntendedElevatorDirection", "down");
      m_desiredPercentSpeed = p * m_downwardPercentSpeed;
    }
    m_sparkMaxMotorPair.setPercentSpeed(m_desiredPercentSpeed);
    SmartDashboard.putNumber("Elevator percent speed: ", m_desiredPercentSpeed);
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
    return new InstantCommand(() -> {m_desiredPosition = m_COLLECTIONPOSITION;});
  }

  public Command goToSafeToExtendPosition(){
    return new InstantCommand(() -> {m_desiredPosition = m_SAFETOEXTENDPOSITION;});
  }

  public Command goToHomePosition(){
    return runOnce(() -> {m_desiredPosition = m_homePosition;});
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_centerStagePositionInches = getCenterStagePositionInches();
    m_distanceToDesiredPosition = getDistanceToDesiredPositionInches();
    SmartDashboard.putNumber("Center stage position inches", m_centerStagePositionInches);
    SmartDashboard.putBoolean("Is close to desired position", isCloseToDesiredPosition());
    SmartDashboard.putNumber("Elev. Desired position", m_desiredPosition);
    SmartDashboard.putNumber("Elev. Distance to desired position inches", m_distanceToDesiredPosition);
    SmartDashboard.putNumber("Elev. Desired speed", m_desiredPercentSpeed);}
}
