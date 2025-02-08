// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Wrist2 extends SubsystemBase {
  private SparkMaxMotor m_motor;
  private double m_percentSpeed;
  private boolean m_isHoldingPosition;
  private double m_positionToHold;

  /** Creates a new Wrist2, the joint between the arm and the shooter2.
   * This is powered by a brushless NEW with a planetary gearbox and chain-drive sprockets
   */
  public Wrist2() {
    m_motor = new SparkMaxMotor(Constants.Wrist2Constants.kCANId,
                                Constants.Wrist2Constants.encoderRotationsPerFinalRotation,
                                Constants.Wrist2Constants.name);
    m_motor.setToBrakeOnIdle(false);
    m_motor.setAndEnableLowerSoftLimit(Constants.Wrist2Constants.lowerSoftLimit); // 0.0
    m_motor.setAndEnableUpperSoftLimit(Constants.Wrist2Constants.upperSoftLimit); // 0.28 or so
    m_percentSpeed = 0.0;
    m_isHoldingPosition = false;
  }

  public void setToBrakeOnIdle (boolean breakOnIdle){
    m_motor.setToBrakeOnIdle(breakOnIdle);
  }

  /**
   * Set speed on scale of [-1,1].  I think positive causes clockwise rotation when viewed from the robot's left.
   * Be very careful - speeds around 0.04 will raise wrist from hanging straight down to about horizontal, where it
   * stalls.  Much higher will let it go to straight up, at which point 0.04 will cause it to move fast.
   * @param percentSpeed - speed on scale of [-1,1]
   */
  public void setPercentSpeed(double percentSpeed){
    disableHoldPosition();
    m_percentSpeed = percentSpeed;
  }

  /**
   * Cut power to wrist motor.  Shooter may droop, relying only on brake mode to hold it in place.
   * Use holdPosition if you want want the wrist to maintain its position.
   */
  public void stop(){
    setPercentSpeed(0.0);
  }

  /**
   * @return position of shooter in number rotations of axle from 0
   */
  public double getPosition(){
    return m_motor.getPosition();
  }

  /**
   * @return velocity in rotations of wrist axle per minute
   */
  public double getVelocity(){
    return m_motor.getVelocity();
  }

  /**
   * Access the SparkMaxMotor object so you can do things that are
   * not explicitly dealt with by Wrist2.  If you do this much,
   * it would be better to add a method to Wrist2 instead.
   * @return the SparkMaxMotor object for the wrist.
   */
  public SparkMaxMotor getSparkMaxMotor(){
    return m_motor;
  }

  /**
   * Reset encoder so current position is considered to be the zero point.
   */
  public void setCurrentPositionAsZeroEncoderPosition(){
    m_motor.setCurrentPositionAsZeroEncoderPosition();
  }

  /**
   * Start trying to maintain wrist's angle at it current position.
   * Position holding will stop when speed is set (to anything).
   */
  public void holdPosition(){
    holdPosition(getPosition());
  }

  /**
   * Stop trying to maintain wrist's angle at its current position.
   */
  public void disableHoldPosition(){
    if (m_isHoldingPosition) {
      System.out.println("Wrist2 no longer is trying to hold its position");
    }
    m_isHoldingPosition = false;
  }

  /**
   * Start trying to maintain wrist's angle.
   * Position holding will stop speed is set (to anything).
   * @param positionToHold angle to hold, in rotations of the wrist.
   */
  public void holdPosition(double positionToHold){
    System.out.println("Wrist2 is starting to try to hold its position");
    m_isHoldingPosition = true;
    m_positionToHold = positionToHold; 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (m_isHoldingPosition && m_positionToHold != 0.0) {
      // bang bang control.  The speeds should depend on the angle from vertical, but we don't know that now
      double positionError = m_positionToHold - getPosition();
      if (positionError > Units.degreesToRotations(0.0)) {
        m_percentSpeed = 0.15 * Math.abs(Units.rotationsToDegrees(positionError));
      } else if (positionError < -Units.degreesToRotations(0.0)){
        m_percentSpeed = -0.01 * Math.abs(Units.rotationsToDegrees(positionError));
      } else {
        m_percentSpeed = 0.0;
      }
    }
    m_motor.setPercentSpeed(m_percentSpeed);
  }
}
