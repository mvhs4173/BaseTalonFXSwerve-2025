// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberServo extends SubsystemBase {

  private Servo m_climberServo;

  /** Creates a new ClimberServo. */
  public ClimberServo(int channel, double startingAngle) {
    m_climberServo = new Servo(channel);
    m_climberServo.setAngle(startingAngle);
  }

  public ClimberServo(int channel){
    m_climberServo = new Servo(channel);
  }

  public void setAngle(double pos) {
    m_climberServo.setAngle(pos);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
