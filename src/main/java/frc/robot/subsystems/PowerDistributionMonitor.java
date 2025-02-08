// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Send voltage and current readings from power distribution board,
 * either the CTRE PDP or the RevRobotics PDH, on the smartdashboard.
 * They must be using the expected CAN numbers - 0 for PDP, 1 for PDH.
 * 
 * To use this, create a PowerDistributionMonitor in RobotContainer.
 * You do not have to use it after creating it (but you must retain
 * its value so the garbage collector does not remove it).
 */
public class PowerDistributionMonitor extends SubsystemBase {
  private final PowerDistribution m_pd = new PowerDistribution();
  private final PowerDistribution.ModuleType m_type = m_pd.getType(); // kCTRE or kRev
  private final int m_numChannels = m_pd.getNumChannels();
  private double m_voltageThreshhold = 7.0;
  private double m_totalCurrentThreshhold = 180.0;
  private int m_underVoltageCount = 0;
  private int m_overTotalCurrentCount = 0;
  /** 
   * Monitor the power distribution board via SmartDashboard.
   * This will also report the number of times the voltage is below 7V
   * and the number of times the total current is over 180A.
   */
  public PowerDistributionMonitor() {}

  /**
   * Monitor power distribution board with user-defined threshholds for counting
   * under-voltage and over-current events.
   * If robot program includes more than one PowerDistribution object you will
   * see CAN errors and all numbers will be reported as zero.
   * @param voltageThreshhold the number of times the input voltage is below this
   * will be reported.
   * @param totalCurrentThreshhold the number of times the total current draw is
   * above this will be reported.
   */
  public PowerDistributionMonitor(double voltageThreshhold, double totalCurrentThreshhold) {
    m_voltageThreshhold = voltageThreshhold;
    m_totalCurrentThreshhold = totalCurrentThreshhold;
  }
 
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double voltage = m_pd.getVoltage();
    if (voltage < m_voltageThreshhold){
      m_underVoltageCount++;
    }
    SmartDashboard.putNumber(m_type + " Voltage", voltage);
    SmartDashboard.putNumber(m_type + " Under " + m_voltageThreshhold + "V. Count", m_underVoltageCount);

    double totalCurrent = m_pd.getTotalCurrent();
    if (totalCurrent > m_totalCurrentThreshhold){
      m_overTotalCurrentCount++;
    }
    SmartDashboard.putNumber(m_type + " Total Current", totalCurrent);
    SmartDashboard.putNumber(m_type + " Over " + m_totalCurrentThreshhold + "A. Count", m_overTotalCurrentCount);
    
    SmartDashboard.putNumber(m_type + " Temperature", m_pd.getTemperature());
    
    for(int channel = 0 ; channel < m_numChannels ; channel++){
      SmartDashboard.putNumber(m_type + " Ch. " + channel, m_pd.getCurrent(channel));
    }
  }
}
