// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pneumatics extends SubsystemBase {
private final DoubleSolenoid m_doubleSolenoid;
private final Compressor m_compressor;
  /** Creates a new Pneumatics. */
  public Pneumatics(int forwardChannel, int reverseChannel) {
    m_compressor = new Compressor(PneumaticsModuleType.REVPH);
    m_doubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, forwardChannel, reverseChannel);
  }

  /*public void makeDoubleSolenoid(int forwardChannel, int reverseChannel){

  }*/

  public void setToForward(){
    m_doubleSolenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void setToReverse(){
    m_doubleSolenoid.set(DoubleSolenoid.Value.kReverse);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
