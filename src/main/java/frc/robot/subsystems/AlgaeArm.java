// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CANId;
import frc.robot.Constants;

public class AlgaeArm extends SubsystemBase {
  private final Solenoid m_solenoid;
  private final SparkMaxMotor m_rollerMotor;
  private final double m_ROLLERINWARDPERCENTSPEED = -1.0; //TODO: adjust these
  private final double m_ROLLEROUTWARDPERCENTSPEED = 1.0; //TODO: adjust these
  private final double m_ROLLERHOLDINGALGAEPERCENTSPEED = -0.2; //TODO: adjust these
  private boolean m_isExtended = false;
  private double m_rollerPercentSpeed = 0.0;
  private double m_algaeCurrent;
  public AlgaeIntakeInfo algaeIntakeInfo;

  /** Creates a new AlgaeArm. */
  public AlgaeArm(CANId rollerCAN) {
    m_solenoid = new Solenoid(Constants.pneumaticsModuleType, 8);
    m_rollerMotor = new SparkMaxMotor(rollerCAN, 5, "Algae Roller");
    m_rollerMotor.setToBrakeOnIdle(true);
    setDefaultCommand(rollerStop());
    SmartDashboard.putBoolean("Algae Roller has hit top speed", false);
    SmartDashboard.putBoolean("Algae Roller has hit high current", false);
    SmartDashboard.putNumber("Roller motor velocity", m_rollerMotor.getVelocity());
    SmartDashboard.putBoolean("Roller At Top Speed", false);
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

  private double getAlgaeCurrent(){
    return m_rollerMotor.getCurrent();
  }

  private void setRollerPercentSpeed(double percent){
    m_rollerMotor.setPercentSpeed(percent);
    m_rollerPercentSpeed = percent;
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

  private class AlgaeIntakeInfo {
    public boolean hasHitTopSpeed;
    public boolean hasHitHighCurrent;
    public Debouncer debouncer;
    public static final double TOP_SPEED = -850; //velocity at which it triggers hasHitTopSpeed
    public static final double HIGH_CURRENT = 50; //Current at which it triggers hasHitHighCurrent, if hasHitTopSpeed
    AlgaeIntakeInfo(){
      hasHitTopSpeed = false;
      hasHitHighCurrent = false;
      debouncer = new Debouncer(0.1);
    }
  }

  public Command rollerIntake(){

   return startRun(()->{algaeIntakeInfo = new AlgaeIntakeInfo();},() -> {
      boolean atTopSpeed = m_rollerMotor.getVelocity() < algaeIntakeInfo.TOP_SPEED;
      SmartDashboard.putBoolean("Roller At Top Speed", atTopSpeed);
      algaeIntakeInfo.hasHitTopSpeed = algaeIntakeInfo.hasHitTopSpeed || atTopSpeed;
      SmartDashboard.putNumber("Roller motor velocity", m_rollerMotor.getVelocity());
      algaeIntakeInfo.hasHitHighCurrent =
        algaeIntakeInfo.hasHitHighCurrent
        || (algaeIntakeInfo.hasHitTopSpeed && algaeIntakeInfo.debouncer.calculate((getAlgaeCurrent() > algaeIntakeInfo.HIGH_CURRENT)));
      SmartDashboard.putBoolean("Algae Roller has hit top speed", algaeIntakeInfo.hasHitTopSpeed);
      SmartDashboard.putBoolean("Algae Roller has hit high current", algaeIntakeInfo.hasHitHighCurrent);
      setRollerPercentSpeed(algaeIntakeInfo.hasHitHighCurrent ? m_ROLLERHOLDINGALGAEPERCENTSPEED : m_ROLLERINWARDPERCENTSPEED);
    }).withName("Algae Roller Intake");


  }

  public Command rollerStop(){
    return run(() -> setRollerPercentSpeed(0.0)).withName("Algae Roller stop (default)");
  }

  public Command rollerPushOut(){
    return run(() -> setRollerPercentSpeed(m_ROLLEROUTWARDPERCENTSPEED));
  }

  /*public Command rollerHoldAlgae(){
    return run(() -> setRollerPercentSpeed(m_ROLLERHOLDINGALGAEPERCENTSPEED));
  }*/

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Is Algae arm extended", m_isExtended);
    SmartDashboard.putNumber("Roller percent speed", m_rollerPercentSpeed);
  }
}
