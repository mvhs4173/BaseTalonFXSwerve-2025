// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shoulder;

public class ShoulderGoToPosition extends Command {
  public static enum Method {
    kRPM,
    kPercentSpeed
  }
  private Method m_method;
  private Shoulder m_shoulder;
  private double m_absoluteValueSpeed;
  private double m_desiredPosition;
  private double m_speed;
  /**
   * Create a new ShouldGoToPosition command.  'speed' can be either RPM or percent power.
   * @param shoulder - a Shoulder object
   * @param method - an enum (ShoulderGoToPosition.Method) with value kRPM or kPercentSpeed that
   * determines how speed is to be interpreted.  kRPM means units are RPM, which is only valid
   * when shoulder is raising the arm, not when shoulder is raising the chassis.  kPercentSpeed
   * means speed is a number in [-1,1].  Negative numbers raise the arm, positive lower it. 
   * @param speed - the absolute value of the RPM or percent speed, depending on method.
   * The sign of the speed will be determined by comparing the current position with the
   * desired position.
   * @param desiredPosition - position to go to, in arm rotations clockwise when looked at from
   * robots left.
   */
  public ShoulderGoToPosition(Shoulder shoulder, Method method, double absoluteValueSpeed, double desiredPosition) {
    m_shoulder = shoulder;
    m_desiredPosition = desiredPosition;
    m_absoluteValueSpeed = absoluteValueSpeed;
    m_method = method;
    addRequirements(m_shoulder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_speed = m_desiredPosition > m_shoulder.getPosition()
      ? m_absoluteValueSpeed
      : -m_absoluteValueSpeed;
    switch (m_method) {
      case kPercentSpeed:
        m_shoulder.setPercentSpeed(m_speed);
        break;
      case kRPM:
        m_shoulder.setRPM(m_speed);
        break; 
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shoulder.setPercentSpeed(0.0);
    //m_shoulder.holdPosition(); // use PID to hold, see if it works
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean finished = m_speed >= 0
      ? m_shoulder.getPosition() >= m_desiredPosition
      : m_shoulder.getPosition() <= m_desiredPosition;
      if (finished) System.out.println("shoulder is finished at " + m_shoulder.getPosition());
    return finished;
  }
}
