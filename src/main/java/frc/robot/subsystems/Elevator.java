// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
  private double m_zeroPosition;
  private double m_tolerance = 0.5; //TODO: adjust these
  private double m_upwardPercentSpeed = 0.2; //TODO: adjust these
  private double m_downwardPercentSpeed = -0.1; //TODO: adjust these
  private final double m_CollectionPosition = 0.0; //TODO: adjust these
  private final double m_L1Position = 10.0; //TODO: adjust these
  private final double m_L2Position = 20.0; //TODO: adjust these
  private final double m_L3Position = 30.0; //TODO: adjust these
  private final double m_L4Position = 40.0; //TODO: adjust these
  private final double m_UpperHeightLimit = 50.0; //TODO: adjust these
  private final double m_LowerHeightLimit = 0.0; //TODO: adjust these

  /** Creates a new Elevator. */
  public Elevator(CANId leftCanId, CANId rightCanId) {
    m_leftMotor = new SparkMaxMotor(leftCanId, 5, "Left Elevator Motor");
    m_rightMotor = new SparkMaxMotor(rightCanId, 5, "Right Elevator Motor");
    m_sparkMaxMotorPair = new SparkMaxMotorPair(m_leftMotor, m_rightMotor, true);
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

  private void goToPositionInches(double desiredPositionInches){
    if(desiredPositionInches >= m_LowerHeightLimit && desiredPositionInches <= m_UpperHeightLimit){
      if(Math.abs(desiredPositionInches - m_centerStagePositionInches) < m_tolerance){
        m_sparkMaxMotorPair.setPercentSpeed(0.0);
      } else if (desiredPositionInches > m_centerStagePositionInches){
        m_sparkMaxMotorPair.setPercentSpeed(m_upwardPercentSpeed);
      } else if (desiredPositionInches < m_centerStagePositionInches){
        m_sparkMaxMotorPair.setPercentSpeed(m_downwardPercentSpeed);
      }
    } else if (desiredPositionInches < m_LowerHeightLimit){
      if(Math.abs(m_LowerHeightLimit - m_centerStagePositionInches) < m_tolerance){
        m_sparkMaxMotorPair.setPercentSpeed(0.0);
      } else if (m_LowerHeightLimit > m_centerStagePositionInches){
        m_sparkMaxMotorPair.setPercentSpeed(m_upwardPercentSpeed);
      } else if (m_LowerHeightLimit < m_centerStagePositionInches){
        m_sparkMaxMotorPair.setPercentSpeed(m_downwardPercentSpeed);
      }
    } else if (desiredPositionInches > m_UpperHeightLimit){
      if(Math.abs(m_UpperHeightLimit - m_centerStagePositionInches) < m_tolerance){
        m_sparkMaxMotorPair.setPercentSpeed(0.0);
      } else if (m_UpperHeightLimit > m_centerStagePositionInches){
        m_sparkMaxMotorPair.setPercentSpeed(m_upwardPercentSpeed);
      } else if (m_UpperHeightLimit < m_centerStagePositionInches){
        m_sparkMaxMotorPair.setPercentSpeed(m_downwardPercentSpeed);
      }
    }
  }

  public Command goToL4Position(){
    return new RunCommand(() -> {goToPositionInches(m_L4Position);});
  }

  public Command goToL3Position(){
    return new RunCommand(() -> {goToPositionInches(m_L3Position);});
  }

  public Command goToL2Position(){
    return new RunCommand(() -> {goToPositionInches(m_L2Position);});
  }

  public Command goToL1Position(){
    return new RunCommand(() -> {goToPositionInches(m_L1Position);});
  }

  public Command goToCollectionPosition(){
    return new RunCommand(() -> {goToPositionInches(m_CollectionPosition);});
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_centerStagePositionInches = getCenterStagePositionInches();
  }
}
