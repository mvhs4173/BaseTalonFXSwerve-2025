// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.text.DecimalFormat;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
//import com.revrobotics.SparkRelativeEncoder;
//import com.revrobotics.CANSparkBase.ControlType;
//import com.revrobotics.CANSparkBase.IdleMode;
//import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.RelativeEncoder;
//import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj.Relay.Direction;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.TuningVariables;

public class SparkMaxMotor extends SubsystemBase {
  private final String m_name;
  private final SparkMax m_CANSparkMax;
  //private final MotorType m_motorType;
  private final RelativeEncoder m_RelativeEncoder;
  private final SparkClosedLoopController m_SparkPIDController;
  private final double m_encoderRotationsPerFinalRotation;
  private double m_zeroEncoderPosition;
  private ClosedLoopSlot m_SmartMotionSlot;
  private DecimalFormat df2 = new DecimalFormat("#.00"); // for 2 digits after decimal in printouts
  private double m_desiredPosition = 0.0;
  private SparkMaxConfig m_config = new SparkMaxConfig();

  
  /** Creates a new SparkMaxBrushless. */
  /*public SparkMaxMotor(int canId, double encoderRotationsPerFinalRotation, String name){
    this(canId, encoderRotationsPerFinalRotation, name);
  }*/
  public SparkMaxMotor(int canId, double encoderRotationsPerFinalRotation, String name) {
    m_name = name;
    m_encoderRotationsPerFinalRotation =  encoderRotationsPerFinalRotation;
    /*if (motorType == null){
      motorType = MotorType.kBrushless;
    }*/
    m_CANSparkMax = new SparkMax(canId, MotorType.kBrushless);
    configure();
    /*m_motorType = motorType;
    if (motorType == MotorType.kBrushless){
      m_RelativeEncoder = m_CANSparkMax.getEncoder();
    } else if (encoderType != null) {
      m_RelativeEncoder = m_CANSparkMax.getEncoder(encoderType, encoderCountsPerRevolution);
    } else {
      m_RelativeEncoder = null;  // force null ptr exception if we try to use encoder, kNoEncoder might be better
    }*/
    
    m_RelativeEncoder = m_CANSparkMax.getEncoder();
    SmartDashboard.putString("relative encoder type = ", m_RelativeEncoder.toString());
    m_SparkPIDController = m_CANSparkMax.getClosedLoopController();
    setCurrentPositionAsZeroEncoderPosition();
  }

  private void configure(){
    m_CANSparkMax.configure(m_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /**
   * @return number of encoder rotations per final rotation for this SparkMaxMotor
   */
  public double getEncoderRotationsPerFinalRotation() {
    return m_encoderRotationsPerFinalRotation;
  }
  public MotorType getMotorType(){
    return MotorType.kBrushless;
  }
  public SparkMax getSparkMax(){
    return m_CANSparkMax;
  }

  /**
   * @param brakeOnIdle - if true, motor will be put in break mode when percentSpeed is 0;
   * if false, motor will put put in coast mode when idle.
   */
  public void setToBrakeOnIdle(boolean brakeOnIdle){
    if (brakeOnIdle){
      m_config
          .idleMode(IdleMode.kBrake);
    }else{ 
      m_config
          .idleMode(IdleMode.kCoast);
    }
    configure();
  }

  // TODO: use 	enableSoftLimit​(CANSparkBase.SoftLimitDirection direction, boolean enable)
  // and 	setSoftLimit​(CANSparkBase.SoftLimitDirection direction, float limit) to limit
  // position.

  /**
   * Cause another SparkMaxMotor to get the same voltage as this motor.
   * This motor will use its set point and feedback from its encoder to set
   * the voltages for all the followers.  Hence we check that their properties
   * match pretty closely.  We assume that all the motors have very similar
   * loads, so the main motor's PID parameters will result in reasonable voltages
   * for them.
   * @param follower- the SparkMaxMotor to mimic the this motor's behavior
   * @param invert- if true, the follower will have the same speed as the main
   * motor but in the opposite direction (use this when motors are at opposite
   * ends of a shaft).  If false the velocities will be identical.
   * @throws Exception- it is a fatal error if the encoder rotations per final
   * rotation or the motor types don't match.
   */
  public void addFollower(SparkMaxMotor follower, boolean invert){
    if (follower.getEncoderRotationsPerFinalRotation() != getEncoderRotationsPerFinalRotation()) {
      throw new Error("follower's encoder rotations per final rotation must match the leader's");
    } else
    if (follower.getMotorType() != getMotorType()){
      throw new Error("follower's motor type must match the leader's");
    } else {
      m_config
          .follow(m_CANSparkMax, invert);
      configure();
    }
  }

  /**
   * Set zero point for encode to its current position.  This may be wanted in case
   * code started before robot was in its initial configuration.
   */
  public void setCurrentPositionAsZeroEncoderPosition(){
    if (m_RelativeEncoder == null){
      System.out.println("No encoder: cannot set current position as zero position!");
    } else {
      System.out.print(m_name + ": Changing zeroEncoderPosition from " + df2.format(m_zeroEncoderPosition));
      m_zeroEncoderPosition = m_RelativeEncoder.getPosition();
      System.out.println(" to " + df2.format(m_zeroEncoderPosition));
    }
  }
  /**
   * @return
   * Position of thing being rotated.  Units = rotations of that thing.
   */
  public double getPosition(){
    // return (m_RelativeEncoder.getPosition() - m_zeroEncoderPosition) / m_encoderRotationsPerFinalRotation;
    return encoderPositionToFinalPosition(m_RelativeEncoder.getPosition());
  }

  /**
   * @return
   * Velocity of thing being rotated.  Units = rotations of that thing per minute
   */
  public double getVelocity(){
    return m_RelativeEncoder.getVelocity() / m_encoderRotationsPerFinalRotation;
  }

  /**
   * @return
   * The encoder associated with this motor controller.  You may use this
   * if you wish to use motor-oriented units for position and velocity.
   */
  public RelativeEncoder getEncoder(){
    return m_RelativeEncoder;
  }

  public String getName(){
    return m_name;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (TuningVariables.debugLevel.getNumber() >= 5.0) {
      SmartDashboard.putNumber(getName() + " pos", getPosition());
      SmartDashboard.putNumber(getName() + " vel", getVelocity());
      SmartDashboard.putNumber(getName() + " amps", m_CANSparkMax.getOutputCurrent());
      SmartDashboard.putNumber(getName() + " desired pos", m_desiredPosition);
    }
  }

  /**
   * @param percent: desired 'speed' on a scale of -1.0 to 1.0
   */
  public void setVoltage(double voltage){
    m_CANSparkMax.setVoltage(voltage);
  }
  public void setPercentSpeed(double percent){
    m_CANSparkMax.set(percent);
  }
  public void setRPM(double rpm){
    double encoderRpm = rpm * m_encoderRotationsPerFinalRotation;
    // System.out.println("setting desired encoder rpm to " + encoderRpm);
    m_SparkPIDController.setReference(encoderRpm, ControlType.kVelocity);
  }
  // We use encoder-centric PID parameters so we can copy them from test/tuning program
  public void setPIDCoefficients(double kP, double kI, double kD, double kIZone, double kFeedForward, double kMinOutput, double kMaxOutput) {
    m_config.closedLoop
        .pid(kP, kI, kD)
        .iZone(kIZone)
        .velocityFF(kFeedForward)
        .outputRange(kMinOutput, kMaxOutput);
  }
  public class PIDCoefficients {
    public double m_kP;
    public double m_kI;
    public double m_kD;
    public double m_kIZone;
    public double m_kFeedForward;
    public double m_kMinOutput;
    public double m_kMaxOutput;
    PIDCoefficients(double kP, double kI, double kD, double kIZone, double kFeedForward, double kMinOutput, double kMaxOutput){
      m_kP = kP; m_kI = kI; m_kD = kD; m_kIZone = kIZone; m_kFeedForward = kFeedForward; m_kMinOutput = kMinOutput; m_kMaxOutput = kMaxOutput;
    }
  }
  public void setPIDCoefficients(PIDCoefficients pidCoef){
    setPIDCoefficients(pidCoef.m_kP, pidCoef.m_kI, pidCoef.m_kD, pidCoef.m_kIZone, pidCoef.m_kFeedForward, pidCoef.m_kMinOutput, pidCoef.m_kMaxOutput);
  }

  // Here we use position, velocities, and acceleration from point of view of thing being rotated.  Time unit is minute. 
  private double encoderPositionToFinalPosition(double encoderPosition){
    return (encoderPosition - m_zeroEncoderPosition) / m_encoderRotationsPerFinalRotation;
  } 
  private double finalPositionToEncoderPosition(double finalPosition){
    return finalPosition * m_encoderRotationsPerFinalRotation + m_zeroEncoderPosition;
  } 
  public void doSmartMotion(double desiredPosition, double maxVelocity, double minVelocity,
    double maxAcceleration, double allowedClosedLoopError){
      m_desiredPosition = desiredPosition;
      /*m_SparkPIDController.setSmartMotionMaxVelocity(maxVelocity * m_encoderRotationsPerFinalRotation, m_SmartMotionSlot);
      m_SparkPIDController.setSmartMotionMinOutputVelocity(minVelocity * m_encoderRotationsPerFinalRotation, m_SmartMotionSlot);
      m_SparkPIDController.setSmartMotionMaxAccel(maxAcceleration * m_encoderRotationsPerFinalRotation, m_SmartMotionSlot);
      m_SparkPIDController.setSmartMotionAllowedClosedLoopError(allowedClosedLoopError/m_encoderRotationsPerFinalRotation, m_SmartMotionSlot); // what units?*/
      m_config.closedLoop
          .smartMotion.maxVelocity(maxVelocity * m_encoderRotationsPerFinalRotation, m_SmartMotionSlot);
      m_config.closedLoop
          .smartMotion.minOutputVelocity(minVelocity * m_encoderRotationsPerFinalRotation, m_SmartMotionSlot);
      m_config.closedLoop
          .smartMotion.maxAcceleration(maxAcceleration * m_encoderRotationsPerFinalRotation, m_SmartMotionSlot);
      m_config.closedLoop
          .smartMotion.allowedClosedLoopError(allowedClosedLoopError/m_encoderRotationsPerFinalRotation, m_SmartMotionSlot);  
      configure();
      //double desiredEncoderPosition = m_desiredPosition * m_encoderRotationsPerFinalRotation + m_zeroEncoderPosition;
      double desiredEncoderPosition = finalPositionToEncoderPosition(m_desiredPosition);
      System.out.println("Going from encoder position " + df2.format(m_RelativeEncoder.getPosition()) + " to " + df2.format(desiredEncoderPosition));
      m_SparkPIDController.setReference(desiredEncoderPosition, SparkMax.ControlType.kSmartMotion);
    }
    /**
     * Stop pushing backward when structure has rotated down to or below minFinalPosition
     * @param minFinalPosition - minimum allowed position.  Units are revolutions of
     * final structure.
     */
    public void setAndEnableLowerSoftLimit(double minFinalPosition){
      double minEncoderPosition = finalPositionToEncoderPosition(minFinalPosition);
      m_config.softLimit.reverseSoftLimit((float)minEncoderPosition);
      m_config.softLimit.reverseSoftLimitEnabled(true);
      configure();
    }
    /**
     * Stop pushing forward when structure has rotated up to or past maxFinalPosition
     * @param maxFinalPosition - maximum allowed position.  Units are revolutions of
     * final structure.
     */
    public void setAndEnableUpperSoftLimit(double maxFinalPosition){
      double maxEncoderPosition = finalPositionToEncoderPosition(maxFinalPosition);
      m_config.softLimit.forwardSoftLimit((float)maxEncoderPosition);
      m_config.softLimit.forwardSoftLimitEnabled(true);
      configure();
    }
    /**
     * Stop respecting soft limits in both directions.  Intended for emergency use or for testing.
     */
    public void disableSoftLimits(){
      System.out.println(m_name + ": disabling soft limits on position");
      m_config.softLimit.forwardSoftLimitEnabled(false);
      m_config.softLimit.reverseSoftLimitEnabled(false);
    }
    /**
     * Resume respecting soft limits in both directions
     */
    public void enableSoftLimits(){
      System.out.println(m_name + ": enabling soft limits on position"); 
      m_config.softLimit.forwardSoftLimitEnabled(true);
      m_config.softLimit.reverseSoftLimitEnabled(true);
    }
    /**
     * Not implemented yet.  The intent is to be still, even if some force is
     * pushing on it.
     */
    public void holdCurrentPosition(){
      // TODO: Use raw (not "smart motion") position control.  Must use different set of PID values.
      // Should we use "slots": one for velocity and one for position? 
      // Also, we may want to use more intelligent feedforward (depending on angle of arm) while holding.
    }
}
