// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gyro extends SubsystemBase {
  /** Creates a new Gyro. */
  private AHRS m_navX;
  private double m_yawOffsetNavX;

  public Gyro() {
    m_navX = new AHRS(NavXComType.kMXP_SPI);
    calibrateNavX();
    setYaw(0.0);
  }
 
  public void setYaw(double degreesCcw){
    m_navX.reset();
    m_yawOffsetNavX = degreesCcw + m_navX.getYaw();
   }

  private double getYawNavX() {
    return -m_navX.getYaw() + m_yawOffsetNavX;
  }
  public double getYaw(){
    return getYawNavX();
  }
  
  public double getRoll(){
    return m_navX.getRoll();
  }
  
  public double getPitch(){
    return m_navX.getPitch();
  }

  /**
   * Make sure gyro is done calibrating before using it
   * @return
   * true if calibration was successful, false if we could not connect to or could not calibrate NavX
   */
  private boolean calibrateNavX() {
    // calibration only needed for NavX
    int nTries = 1;
    boolean retval = true;
    while (m_navX.isCalibrating() && nTries<100) { //wait to zero yaw if calibration is still running
      try {
        Thread.sleep(20);
        System.out.println("----calibrating gyro---- " + nTries);
      } catch (InterruptedException e) {

      }
      nTries++;
      if (nTries >= 50 && nTries%10==0) {
        System.out.println("Having trouble calibrating NavX");
      }
    }
    try {
      Thread.sleep(60); // sometimes isConnected returns false immediately after calibration
    } catch (InterruptedException e) {
      // do nothing
    }
    if (m_navX.isCalibrating()) {
      System.out.println("Could not calibrate NavX");
      retval = false;
    } else if (!m_navX.isConnected()) {
      System.out.println("NavX is not connected (is SPI dip switch not ON?)");
      retval = false;
    }
    return retval;
  }
 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
      double navXYaw = getYawNavX();
      SmartDashboard.putNumber("NavX Yaw", navXYaw);
  }

}