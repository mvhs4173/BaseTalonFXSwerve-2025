// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  private PhotonCamera camera;
  private PhotonPipelineResult m_result;
  private boolean m_hasTargets;
  /** Creates a new Vision. */ 
  public Vision(String cameraName) {
    camera = new PhotonCamera(cameraName);
  }
  public boolean hasTargets(){
    return m_hasTargets;
  }
  public double getYaw(){
    double yaw;
    if (m_hasTargets){
      PhotonTrackedTarget target = m_result.getBestTarget();
      yaw = target.getYaw();
    } else {
      yaw = 0.0;
    }
    return yaw;
  }
  public double getYaw(PhotonTrackedTarget target){
    return target.getYaw();
  }
  public PhotonTrackedTarget getBestAprilTag(){
    if (m_hasTargets){
      return m_result.getBestTarget();
    } 
    return null;
  }
  public PhotonTrackedTarget getAprilTag(int id){
    if (m_hasTargets){
      List<PhotonTrackedTarget> targets = m_result.getTargets();
      for(PhotonTrackedTarget target: targets){
        if (target.getFiducialId() == id) {
          return target;
        } 
      }
    }
    return null;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_result = camera.getLatestResult();
    m_hasTargets = m_result.hasTargets();
    SmartDashboard.putBoolean("Has Targets", m_hasTargets);
    SmartDashboard.putNumber("yaw", getYaw());
    
  }
}
