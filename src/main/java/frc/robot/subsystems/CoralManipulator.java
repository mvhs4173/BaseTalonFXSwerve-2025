// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralManipulator extends SubsystemBase {
  private Elevator m_Elevator;
  private CoralArm m_CoralArm;
  /** Creates a new CoralManipulator. */
  public CoralManipulator(Elevator elevator, CoralArm coralArm) {
    m_Elevator = elevator;
    m_CoralArm = coralArm;
  }

  /*go to safe extension position, extend, go to collection position, roll in (collect),
   go to safe extension position, retract
   for the purpose of grabbing a horizontal-laying coral on the floor directly in front of the robot*/
  public Command collectCoral(){
    return Commands.sequence(
      m_Elevator.goToSafeToExtendPosition().until(() -> m_Elevator.isCloseToDesiredPosition()).withTimeout(2.0),
      m_CoralArm.armExtend().withTimeout(2.0),
      m_Elevator.goToCollectionPosition().until(() -> m_Elevator.isCloseToDesiredPosition()).withTimeout(2.0),
      m_CoralArm.rollerIntake().until(() -> m_CoralArm.isCoralInIntake()).withTimeout(2.0),
      m_Elevator.goToSafeToExtendPosition().until(() -> m_Elevator.isCloseToDesiredPosition()).withTimeout(2.0),
      m_CoralArm.armRetract()
    ).withName("Collect Coral command");
  }

  /*go to L4 position, extend arm, rotate arm
    */
    public Command goToL4ScoringPosition(){
      return Commands.sequence(
        m_Elevator.goToL4Position().until(() -> m_Elevator.isCloseToDesiredPosition()).withTimeout(5.0),
        m_CoralArm.armExtend().withTimeout(5.0)
        //m_CoralArm.wristGoToPosition(0.25).until(() -> m_CoralArm.isWristVertical()).withTimeout(2.0)
      );
    }

    /*go to L3 position, extend arm, rotate arm
    */
  public Command goToL3ScoringPosition(){
    return Commands.sequence(
      m_Elevator.goToL3Position().until(() -> m_Elevator.isCloseToDesiredPosition()).withTimeout(3.0),
      m_CoralArm.armExtend().withTimeout(2.0),
      m_CoralArm.wristGoToPosition(0.25).until(() -> m_CoralArm.isWristVertical()).withTimeout(2.0)
    );
  }

  /*go to L2 position, extend arm, rotate arm
    */
  public Command goToL2ScoringPosition(){
    return Commands.sequence(
      m_Elevator.goToL2Position().until(() -> m_Elevator.isCloseToDesiredPosition()).withTimeout(3.0),
      m_CoralArm.armExtend().withTimeout(2.0),
      m_CoralArm.wristGoToPosition(0.25).until(() -> m_CoralArm.isWristVertical()).withTimeout(2.0)
    );
  }

  /*go to L1 position, extend arm -- wrist rotation not neccesary because it is the trough*/
  public Command goToL1TroughScoringPosition(){
    return Commands.sequence(
      m_Elevator.goToL1Position().until(() -> m_Elevator.isCloseToDesiredPosition()).withTimeout(3.0),
      m_CoralArm.armExtend().withTimeout(2.0)
    );
  }

  /*drop the set amount. Amount is set in Elevator.java as the value: m_DISTANCETOLOWERTOSCORE*/
  public Command dropToScoreOnReef(){
    return Commands.sequence(
      m_Elevator.goToLowerPosition()
    );
  }

  /*make roller go outwards */
  public Command pushOutWithRollers(){
    return Commands.sequence(
      m_CoralArm.rollerPushOut()
    );
  }

  public Command goToHome(){
    return Commands.sequence(
      m_Elevator.goToSafeToExtendPosition().withTimeout(2),
      m_CoralArm.armRetract(),
      m_Elevator.goToHomePosition()
    );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
