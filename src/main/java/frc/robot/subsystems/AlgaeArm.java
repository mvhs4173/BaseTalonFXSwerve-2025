// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CANId;
import frc.robot.Constants;

public class AlgaeArm extends SubsystemBase {
  private final Solenoid m_solenoid;
  private final SparkMaxMotor m_rollerMotor;
  private final double m_ROLLERINWARDPERCENTSPEED = 1.0; //TODO: adjust these
  private final double m_ROLLEROUTWARDPERCENTSPEED = -1.0; //TODO: adjust these
  private final double m_ROLLERHOLDINGALGAEPERCENTSPEED = 0.5; //TODO: adjust these
  private boolean m_isExtended = false;

  /** Creates a new AlgaeArm. */
  public AlgaeArm(CANId rollerCAN) {
    m_solenoid = new Solenoid(Constants.pneumaticsModuleType, 8);
    m_rollerMotor = new SparkMaxMotor(rollerCAN, 5, "Algae Roller");
  }

  private void extend(){
    if(!m_isExtended){
      m_solenoid.set(true);
      m_isExtended = true;
    }
  }

  private void retract(){
    if(m_isExtended){
      m_solenoid.set(false);
      m_isExtended = false;
    }
  }

  private void setRollerPercentSpeed(double percent){
    m_rollerMotor.setPercentSpeed(percent);
  }

  private void retractAndSetPercentSpeedToZero(){
    retract();
    setRollerPercentSpeed(0.0);
  }

  public Command extendArm(){
    return runOnce(() -> extend());
  }

  //as well as retracting arm, also sets the roller's percent speed to zero
  public Command retractArm(){
    return runOnce(() -> retractAndSetPercentSpeedToZero());
  }

  public Command rollerIntake(){
    return new RunCommand(() -> setRollerPercentSpeed(m_ROLLERINWARDPERCENTSPEED));
  }

  public Command rollerPushOut(){
    return new RunCommand(() -> setRollerPercentSpeed(m_ROLLEROUTWARDPERCENTSPEED));
  }

  public Command rollerHoldAlgae(){
    return new RunCommand(() -> setRollerPercentSpeed(m_ROLLERHOLDINGALGAEPERCENTSPEED));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
