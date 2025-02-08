// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class BeamBreakSensor extends SubsystemBase {
  
  private OnOffSwitch m_sensor;
  
  
  /** Creates a new BeamBreakSensor. */
  public BeamBreakSensor() {
    m_sensor = new OnOffSwitch(Constants.BeamBreakSensorConstants.channel, Constants.BeamBreakSensorConstants.normallyOpen, Constants.BeamBreakSensorConstants.name);
  }

  public boolean noteIsInShooter(){
    return m_sensor.isActivated();
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Beam Break Active", noteIsInShooter());
  }
}
