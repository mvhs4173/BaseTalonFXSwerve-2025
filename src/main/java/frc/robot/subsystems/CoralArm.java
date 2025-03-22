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

  private final double m_ROLLER_INWARD_PERCENT_SPEED =  -0.35; 
  private final double m_ROLLER_OUTWARD_PERCENT_SPEED = +0.35; 

  private boolean m_isExtended = false;
  //private boolean m_isCoralInIntake = false;
  private boolean m_isWristVertical;
  private boolean m_isWristHorizontal;
  private double m_rollerCurrentDraw;

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
    m_wristMotor = new SparkMaxMotor(wristCAN, 5.0*5.0, "Wrist Motor");
    m_rollerMotor = new SparkMaxMotor(rollerCAN, 5, "Roller Motor");
    m_wristMotor.setToBrakeOnIdle(true);
    m_rollerMotor.setToBrakeOnIdle(true);
    m_CoralDetectionSensor = new OnOffSwitch(OnOffSwitchChannel, true, "Coral Detection Sensor");
    setDefaultCommand(run(() -> wristGoTowardsDesiredPosition()));
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
      m_wristMotor.setPercentSpeed(percent); 
  }

  private void wristSetDesiredPosition(double desiredPositionRotations){
    m_wristDesiredPositionRotations = MathUtil.clamp(desiredPositionRotations, m_WRIST_LOWER_POSITION_LIMIT, m_WRIST_UPPER_POSITION_LIMIT);
  }
  /**
   * Set wrist motor speed to head towards the m_wristDesiredPositionRotations.
   * We expect the speed to be sent to the motor controller by periodic().
   */
  private void wristGoTowardsDesiredPosition(){
    double error = m_wristDesiredPositionRotations - m_wristPositionRotations; // current position is updated in periodic()
    // bang-bang control:
    if(Math.abs(error) <= m_WRIST_POSITION_TOLERANCE){
      m_wristDesiredPercentSpeed = 0.0;
    } else if(error > 0.0){
      m_wristDesiredPercentSpeed = m_WRIST_ABS_MAX_PERCENT_SPEED;
    } else {
      m_wristDesiredPercentSpeed = -m_WRIST_ABS_MAX_PERCENT_SPEED;
    }
  }

  public Command wristGoToPositionAndHold(double desiredPositionRotations){
    return startRun(
      () -> {
        wristSetDesiredPosition(desiredPositionRotations);
        System.out.println("wrist go to " + desiredPositionRotations + " and hold");
      },
      () -> {
        wristGoTowardsDesiredPosition();
      });
  }

  public Command wristGoToPositionAndFinish(double desiredPositionRotations){
    System.out.println("arg=" + desiredPositionRotations);
    return wristGoToPositionAndHold(desiredPositionRotations).until(() -> isWristAtDesiredPosition());
  }

  public Command wristGoToHorizontalAndFinish(){
    return wristGoToPositionAndFinish(0.0).withName("wristGoToHorizontal");
  }

  public Command wristGoToVerticalAndFinish(){
    return wristGoToPositionAndFinish(0.25).withName("wristGoToVertical");
  }

  private double getCoralRollerCurrent(){
    return m_rollerMotor.getCurrent();
  }

   private class CoralIntakeInfo {
    public boolean hasHitCruisingSpeed;
    public boolean hasHitHighCurrent;
    public Debouncer debouncer;
    public static final double CRUISING_SPEED = 200; //velocity at which it triggers hasHitCruisingSpeed
    public static final double HIGH_CURRENT = 50; //Current at which it triggers hasHitHighCurrent, if hasHitCruisingSpeed
    CoralIntakeInfo(){
      hasHitCruisingSpeed = false;
      hasHitHighCurrent = false;
      debouncer = new Debouncer(0.1);
    }
  }


  public Command rollerIntake(){
    return startRun(
    () -> {
      m_coralIntakeInfo = new CoralIntakeInfo();
      setRollerPercentSpeed(m_ROLLER_INWARD_PERCENT_SPEED);
    },
    () -> {
      m_coralIntakeInfo.hasHitCruisingSpeed =
        m_coralIntakeInfo.hasHitCruisingSpeed
        || (m_rollerMotor.getVelocity() > CoralIntakeInfo.CRUISING_SPEED);
      if (m_coralIntakeInfo.hasHitCruisingSpeed) {
        m_coralIntakeInfo.hasHitHighCurrent =
          m_coralIntakeInfo.hasHitHighCurrent
          || m_coralIntakeInfo.debouncer.calculate((getCoralRollerCurrent() > CoralIntakeInfo.HIGH_CURRENT));
      }
    })
    .until(() -> m_coralIntakeInfo.hasHitHighCurrent)
    .finallyDo(() -> setRollerPercentSpeed(0.0))
    .withName("Coral Roller Intake");

  }

  public Command rollerPushOut(){
    return run(() -> {
      setRollerPercentSpeed(m_ROLLER_OUTWARD_PERCENT_SPEED);
    })
    .finallyDo(() -> setRollerPercentSpeed(0.0));
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

  public boolean isWristAtDesiredPosition(){
    return Math.abs(m_wristDesiredPositionRotations - m_wristPositionRotations) < m_WRIST_POSITION_TOLERANCE;
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
    m_rollerCurrentDraw = getCoralRollerCurrent();
    // m_isCoralInIntake = isCoralInIntake();
    m_isWristVertical = isWristVertical();
    m_isWristHorizontal = isWristHorizontal();
    setWristPercentSpeed(m_wristDesiredPercentSpeed);

    SmartDashboard.putBoolean("Wrist is vertical", m_isWristVertical);
    SmartDashboard.putBoolean("Wrist is horizontal", m_isWristHorizontal);
    //SmartDashboard.putBoolean("Is Coral in intake", m_isCoralInIntake);
    
    SmartDashboard.putBoolean("Coral arm is extended", m_isExtended);
    SmartDashboard.putNumber("Coral roller current", m_rollerCurrentDraw);
    SmartDashboard.putNumber("Coral roller velocity", m_rollerMotor.getVelocity());

    SmartDashboard.putNumber("Wrist position", m_wristPositionRotations);
    SmartDashboard.putNumber("Wrist desired position", m_wristDesiredPositionRotations);
    SmartDashboard.putNumber("Wrist desired speed", m_wristDesiredPercentSpeed);
  }
}
