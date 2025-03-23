// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.CommandLogger;

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
    Command safeToExtArmPose = m_Elevator.goToSafeToExtendPosition().andThen(Commands.waitUntil(()->m_Elevator.isCloseToDesiredPosition()).withTimeout(2));
    Command ExtendArm = m_CoralArm.armExtend().andThen(Commands.waitSeconds(1));
    Command WristHorizontal = m_CoralArm.wristGoToHorizontalAndFinish().withTimeout(2);
    Command LowerElevator = m_Elevator.goToCollectionPosition().andThen(Commands.waitUntil(()->m_Elevator.isCloseToDesiredPosition()).withTimeout(2));
    Command RollIntake = m_CoralArm.rollerIntake().until(()->m_CoralArm.isCoralInIntake()).withTimeout(5);
    return Commands.sequence(
    CommandLogger.logCommand(safeToExtArmPose, "safeToExtArmPose"),
    CommandLogger.logCommand(ExtendArm, "ExtendArm"),
    CommandLogger.logCommand(WristHorizontal, "Wrist go to horizontal"),
    CommandLogger.logCommand(LowerElevator, "LowerElevator"),
    CommandLogger.logCommand(RollIntake, "RollIntake")
      //m_Elevator.goToSafeToExtendPosition()
        //.until(() -> m_Elevator.isCloseToDesiredPosition()),
        //.withTimeout(2.0),
      // m_CoralArm
      //   .armExtend()
      //   .withTimeout(1.0),
      // m_Elevator
      //   .goToCollectionPosition()
      //   .until(() -> m_Elevator.isCloseToDesiredPosition()).
      //   withTimeout(2.0),
      // m_CoralArm
      //   .rollerIntake()
      //   .until(() -> m_CoralArm.isCoralInIntake())
      //   .withTimeout(5.0) // was 2
      /*m_Elevator.goToSafeToExtendPosition()
        .until(() -> m_Elevator.isCloseToDesiredPosition())
        .withTimeout(2.0),
      m_CoralArm.armRetract()*/
    ).withName("Collect Coral command");
  }

  /*go to L4 position, extend arm, rotate arm
    */
    public Command goToL4ScoringPosition(){
      return Commands.sequence(
        m_Elevator.goToL4Position().until(() -> m_Elevator.isCloseToDesiredPosition()).withTimeout(3.0),
        m_CoralArm.armExtend().withTimeout(2.0),
        m_CoralArm.wristGoToVerticalAndFinish().withTimeout(2.0)
      ).withName("L4");
    }

    /*go to L3 position, extend arm, rotate arm
    */
  public Command goToL3ScoringPosition(){
    return Commands.sequence(
      m_Elevator.goToL3Position().until(() -> m_Elevator.isCloseToDesiredPosition()).withTimeout(3.0),
      m_CoralArm.armExtend().withTimeout(2.0),
      m_CoralArm.wristGoToVerticalAndFinish().withTimeout(2.0)
    ).withName("L3");
  }

  /*go to L2 position, extend arm, rotate arm
    */
  public Command goToL2ScoringPosition(){
    return Commands.sequence(
      m_Elevator.goToL2Position().until(() -> m_Elevator.isCloseToDesiredPosition()).withTimeout(3.0),
      m_CoralArm.armExtend().withTimeout(2.0),
      m_CoralArm.wristGoToVerticalAndFinish().withTimeout(2.0)
    ).withName("L2");
  }

  /*go to L1 position, extend arm -- wrist rotation not neccesary because it is the trough*/
  public Command goToL1TroughScoringPosition(){
    return Commands.sequence(
      m_Elevator.goToL1Position().until(() -> m_Elevator.isCloseToDesiredPosition()).withTimeout(3.0),
      m_CoralArm.armExtend().withTimeout(2.0),
      m_CoralArm.wristGoToHorizontalAndFinish().withTimeout(2.0)
    ).withName("L1");
  }

  /*drop the set amount. Amount is set in Elevator.java as the value: m_DISTANCETOLOWERTOSCORE*/
  public Command dropToScoreOnReef(){
    return Commands.sequence(
      m_Elevator.goToLowerPosition()
    ).withName("dropToScore");
  }

  /*make roller go outwards */
  public Command pushOutWithRollers(){
    return Commands.sequence(
      m_CoralArm.rollerPushOut()
    ).withName("PushOut");
  }

  public Command goToHome(){
    return Commands.sequence(
      m_CoralArm.wristGoToHorizontalAndFinish().withTimeout(2.0),
      m_CoralArm.armRetract(),
      m_Elevator.goToHomePosition()
    ).withName("goHome");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
