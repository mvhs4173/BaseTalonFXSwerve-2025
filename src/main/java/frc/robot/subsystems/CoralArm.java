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
//import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.CANId;
import frc.robot.Constants;

public class CoralArm extends SubsystemBase {
  /*Need to know: state of cylinders, wrist rotation, roller velocity, limits of motors*/
  private final Solenoid m_Solenoid;
  private final SparkMaxMotor m_wristMotor;
  private final SparkMaxMotor m_rollerMotor;
  private final OnOffSwitch m_CoralDetectionSensor;

  private final double m_ROLLER_INWARD_PERCENT_SPEED =  +0.35; 
  private final double m_ROLLER_OUTWARD_PERCENT_SPEED = -0.35; 

  private boolean m_isExtended = false;
  private boolean m_isCoralInIntake = false;
  private boolean m_isWristVertical;
  private boolean m_isWristHorizontal;
  // private double m_rollerCurrentDraw;

  private final double m_WRIST_LOWER_POSITION_LIMIT =  0.00;
  private final double m_WRIST_UPPER_POSITION_LIMIT = +0.25;
  private final double m_WRIST_POSITION_TOLERANCE = (2.0 / 360.0); // May need tuning 
  private double m_wristDesiredPositionRotations = 0.0;
  private double m_wristPositionRotations;

  private final double m_WRIST_ABS_MAX_PERCENT_SPEED = 0.10; //TODO: adjust these
  private double m_wristDesiredPercentSpeed = 0.0;

  private CoralIntakeInfo m_coralIntakeInfo;

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
    if (Math.abs(m_wristPositionRotations) < m_WRIST_LOWER_POSITION_LIMIT){
      m_wristMotor.setPercentSpeed(percent);
    }
  }

  public Command wristGoToPosition(double desiredPositionRotations){
    m_wristDesiredPositionRotations = MathUtil.clamp(desiredPositionRotations, m_WRIST_LOWER_POSITION_LIMIT, m_WRIST_UPPER_POSITION_LIMIT);
    return run(() -> {
      double error = m_wristDesiredPositionRotations - m_wristPositionRotations; // current position is updated in periodic()
      // bang-bang control:
      if(Math.abs(error) <= m_WRIST_POSITION_TOLERANCE){
        m_wristDesiredPercentSpeed = 0.0;
      } else if(error > 0.0){
        m_wristDesiredPercentSpeed = m_WRIST_ABS_MAX_PERCENT_SPEED;
      } else {
        m_wristDesiredPercentSpeed = -m_WRIST_ABS_MAX_PERCENT_SPEED;
      }
      // expect speed to be set in periodic()
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
      setRollerPercentSpeed(m_coralIntakeInfo.hasHitHighCurrent ? 0.0 : m_ROLLER_INWARD_PERCENT_SPEED);
    }).withName("Algae Roller Intake");

  }

  public Command rollerPushOut(){
    return run(() -> {
      setRollerPercentSpeed(m_ROLLER_OUTWARD_PERCENT_SPEED);
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
    return Math.abs(0.25 - m_wristPositionRotations) < m_WRIST_POSITION_TOLERANCE;
  }

  public boolean isWristHorizontal(){
    return Math.abs(0.0 - m_wristPositionRotations) < m_WRIST_POSITION_TOLERANCE;
  }
  /*Triggers: coral collected, arm extended, wrist vertical,
   wrist horizontal, arm retracted*/
  //public Trigger isCoralInIntakeTrigger = new Trigger(() -> m_isCoralInIntake);
  //public Trigger isArmExtendedTrigger = new Trigger(() -> m_isExtended);
  //public Trigger isWristVerticalTrigger = new Trigger(() -> m_isWristVertical);
  //public Trigger isWristHorizontalTrigger = new Trigger(() -> m_isWristHorizontal);
  //public Trigger isArmRetractedTrigger = new Trigger(() -> !m_isExtended);




  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_wristPositionRotations = m_wristMotor.getPosition();
    m_isCoralInIntake = isCoralInIntake();
    m_isWristVertical = isWristVertical();
    m_isWristHorizontal = isWristHorizontal();
    setWristPercentSpeed(m_wristDesiredPercentSpeed);

    SmartDashboard.putBoolean("Is wrist Vertical", m_isWristVertical);
    SmartDashboard.putBoolean("Is wrist Horizontal", m_isWristHorizontal);
    SmartDashboard.putBoolean("Is Coral in intake", m_isCoralInIntake);
    SmartDashboard.putNumber("Wrist Position", m_wristPositionRotations);
    SmartDashboard.putBoolean("Is coral arm extended", m_isExtended);
    SmartDashboard.putNumber("Wrist desired position", m_wristDesiredPositionRotations);
  }
}
