// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.CANId;
import frc.robot.Constants;

public class CoralArm extends SubsystemBase {
  /*Need to know: state of cylinders, wrist rotation, roller velocity, limits of motors*/
  private final Solenoid m_Solenoid;
  private final SparkMaxMotor m_wristMotor;
  private final SparkMaxMotor m_rollerMotor;
  private final OnOffSwitch m_CoralDetectionSensor;
  private final double m_wristMotorRotationLimit = 0.25;
  private final double m_ROLLERINWARDPERCENTSPEED = 0.35; 
  private final double m_ROLLEROUTWARDPERCENTSPEED = -0.35; 
  private final double m_wristPercentSpeed = 0.1; //TODO: adjust these
  private double m_wristDesiredPercentSpeed = 0.0;
  private boolean m_isExtended = false;
  private double m_wristPosition;
  private boolean m_isCoralInIntake;
  private boolean m_isWristVertical;
  private boolean m_isWristHorizontal;
  private double m_rollerCurrentDraw;
  private double m_tolerance = (2.0 / 360.0); // May need tuning
  private CoralIntakeInfo m_coralIntakeInfo;
  private double m_wristDesiredPosition = 0.0;

  

  /** Creates a new CoralArm. */
  public CoralArm(CANId wristCAN, CANId rollerCAN, int OnOffSwitchChannel) {
    m_Solenoid = new Solenoid(Constants.pneumaticsModuleType, 0);
    m_wristMotor = new SparkMaxMotor(wristCAN, (5*5), "Wrist Motor");
    m_rollerMotor = new SparkMaxMotor(rollerCAN, 5, "Roller Motor");
    m_wristMotor.setToBrakeOnIdle(true);
    m_rollerMotor.setToBrakeOnIdle(true);
    m_CoralDetectionSensor = new OnOffSwitch(OnOffSwitchChannel, true, "Coral Detection Sensor");
  }

  private void extend(){
    if(!m_isExtended){
      m_Solenoid.set(true);
      m_isExtended = true;
    }
  }

  private void retract(){
    if(m_isExtended){
      m_Solenoid.set(false);
      m_isExtended = false;
    }
  }

  private void setRollerPercentSpeed(double percent){
    m_rollerMotor.setPercentSpeed(percent);
  }

  private void setWristPercentSpeed(double percent){
    if (Math.abs(m_wristPosition) < m_wristMotorRotationLimit){
      m_wristMotor.setPercentSpeed(percent);
    }
  }

  public Command wristGoToPosition(double r_desiredPositionRotations){
    
    return run(() -> {
      m_wristDesiredPosition = r_desiredPositionRotations;
      double desiredPositionRotations = MathUtil.clamp(r_desiredPositionRotations, 0, 90.0 / 360.0);
      if(Math.abs(desiredPositionRotations) > m_wristMotorRotationLimit){
        if(Math.abs(m_wristMotorRotationLimit - m_wristPosition) < m_tolerance){
          m_wristDesiredPercentSpeed = 0.0;
        } else if(m_wristMotorRotationLimit > m_wristPosition){
          m_wristDesiredPercentSpeed = m_wristPercentSpeed;
        } else if(m_wristMotorRotationLimit < m_wristPosition){
          m_wristDesiredPercentSpeed = -m_wristPercentSpeed;
        }
       } else {
        if(Math.abs(desiredPositionRotations - m_wristPosition) < m_tolerance){
          m_wristDesiredPercentSpeed = 0.0;
        } else if(desiredPositionRotations > m_wristPosition){
          m_wristDesiredPercentSpeed = m_wristPercentSpeed;
        } else if(desiredPositionRotations < m_wristPosition){
          m_wristDesiredPercentSpeed = -m_wristPercentSpeed;
        }
      }
    });

  }

  private double getCoralRollerCurrent(){
    return m_rollerMotor.getCurrent();
  }

   private class CoralIntakeInfo {
    public boolean hasHitTopSpeed;
    public boolean hasHitHighCurrent;
    public Debouncer debouncer;
    public static final double TOP_SPEED = 200; //velocity at which it triggers hasHitTopSpeed
    public static final double HIGH_CURRENT = 50; //Current at which it triggers hasHitHighCurrent, if hasHitTopSpeed
    CoralIntakeInfo(){
      hasHitTopSpeed = false;
      hasHitHighCurrent = false;
      debouncer = new Debouncer(0.1);
    }
  }


  public Command rollerIntake(){
    return startRun(
    ()->{m_coralIntakeInfo = new CoralIntakeInfo();},
    () -> {
      m_coralIntakeInfo.hasHitTopSpeed = m_coralIntakeInfo.hasHitTopSpeed || (m_rollerMotor.getVelocity() < CoralIntakeInfo.TOP_SPEED);
      m_coralIntakeInfo.hasHitHighCurrent =
      m_coralIntakeInfo.hasHitHighCurrent
        || (m_coralIntakeInfo.hasHitTopSpeed && m_coralIntakeInfo.debouncer.calculate((getCoralRollerCurrent() > CoralIntakeInfo.HIGH_CURRENT)));
      setRollerPercentSpeed(m_coralIntakeInfo.hasHitHighCurrent ? 0.0 : m_ROLLERINWARDPERCENTSPEED);
    }).withName("Algae Roller Intake");

  }

  public Command rollerPushOut(){
    return run(() -> {
      setRollerPercentSpeed(m_ROLLEROUTWARDPERCENTSPEED);
    });
  }

  public Command armExtend(){
    return runOnce(() -> {
      extend();
    });
  }

  public Command armRetract(){
    return runOnce(() -> {
      retract();
    });
  }

  public boolean isCoralInIntake(){
    return m_CoralDetectionSensor.isActivated();
  }

  public boolean isWristVertical(){
    if (Math.abs(0.25 - m_wristPosition) < m_tolerance) {
      return true;
    } else {
      return false;
    }
  }

  public boolean isWristHorizontal(){
    if (Math.abs(0.0 - m_wristPosition) < m_tolerance) {
      return true;
    } else {
      return false;
    }
  }
  /*Triggers: coral collected, arm extended, wrist vertical,
   wrist horizontal, arm retracted*/
  public Trigger isCoralInIntake = new Trigger(() -> m_isCoralInIntake);
  public Trigger isArmExtended = new Trigger(() -> m_isExtended);
  public Trigger isWristVertical = new Trigger(() -> m_isWristVertical);
  public Trigger isWristHorizontal = new Trigger(() -> m_isWristHorizontal);
  public Trigger isArmRetracted = new Trigger(() -> !m_isExtended);




  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_wristPosition = m_wristMotor.getPosition();
    m_isCoralInIntake = isCoralInIntake();
    m_isWristVertical = isWristVertical();
    m_isWristHorizontal = isWristHorizontal();
    setWristPercentSpeed(m_wristDesiredPercentSpeed);

    SmartDashboard.putBoolean("Is wrist Vertical", m_isWristVertical);
    SmartDashboard.putBoolean("Is wrist Horizontal", m_isWristHorizontal);
    SmartDashboard.putBoolean("Is Coral in intake", m_isCoralInIntake);
    SmartDashboard.putNumber("Wrist Position", m_wristPosition);
    SmartDashboard.putBoolean("Is coral arm extended", m_isExtended);
    SmartDashboard.putNumber("Wrist desired position", m_wristDesiredPosition);
  }
}
