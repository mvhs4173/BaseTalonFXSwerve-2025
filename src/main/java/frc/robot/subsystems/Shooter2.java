// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
/**
 * The shooter built for the second competition.  Has top and bottom rollers
 * at its mouth and a roller near the back to pull the note just out of the range
 * of the main intake motors so the main motors can get up to speed before
 * contacting the note.
 */
public class Shooter2 extends SubsystemBase {
  // For all motors, positive voltage intakes, negative shoots.
  SparkMaxMotor m_lowerRoller;
  SparkMaxMotor m_upperRoller;
  SparkMaxMotor m_indexer;
  /** Creates a new Shooter2. */
  public Shooter2() {
    m_lowerRoller = new SparkMaxMotor(
      Constants.Shooter2Constants.LowerRoller.kCANId,
      Constants.Shooter2Constants.LowerRoller.encoderRotationsPerFinalRotation,
      Constants.Shooter2Constants.LowerRoller.name);
    m_upperRoller = new SparkMaxMotor(
      Constants.Shooter2Constants.UpperRoller.kCANId,
      Constants.Shooter2Constants.UpperRoller.encoderRotationsPerFinalRotation,
      Constants.Shooter2Constants.UpperRoller.name);
    m_indexer = new SparkMaxMotor(
      Constants.Shooter2Constants.Indexer.kCANId,
      Constants.Shooter2Constants.Indexer.encoderRotationsPerFinalRotation,
      Constants.Shooter2Constants.Indexer.name);
  }

  /** 
   * Set speed of front rollers for intake
   * @param percentSpeed - speed on a scale of [0,1].  Bigger number mean faster intake.
  */
  public void setMainRollerPercentSpeedForIntake(double percentSpeed){
    m_lowerRoller.setPercentSpeed(percentSpeed);
    m_upperRoller.setPercentSpeed(percentSpeed);
  }
  /**
   * Set speed of rear roller (the "indexer") for intake 
   * @param percentSpeed
   */
  public void setIndexerPercentSpeedForIntake(double percentSpeed){
    m_indexer.setPercentSpeed(percentSpeed);
  }

  /** 
   * Set speed of front rollers for shooting
   * @param percentSpeed - speed on a scale of [0,1].  Bigger number mean faster shot.
  */
  public void setMainRollerPercentSpeedForShooting(double percentSpeed){
    percentSpeed = -percentSpeed;
    m_lowerRoller.setPercentSpeed(percentSpeed);
    m_upperRoller.setPercentSpeed(percentSpeed);
  }
  /**
   * Set speed of rear roller (the "indexer") for shooting
   * @param percentSpeed - speed on a scale of [0,1].  Bigger number means faster shot.
   */
  public void setIndexerPercentSpeedForShooting(double percentSpeed){
    percentSpeed = -percentSpeed;
    m_indexer.setPercentSpeed(percentSpeed);
  }
  /**
   * Cut power to main rollers
   */
  public void stopMainRoller(){
    setMainRollerPercentSpeedForShooting(0.0);
  }

  /** Cut power to indexer */
  public void stopIndexer(){
    setIndexerPercentSpeedForShooting(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command shoot2ForSpeakerCommand(){
    Command command =
          new InstantCommand(() -> setMainRollerPercentSpeedForShooting(0.9))
              .andThen(new WaitCommand(0.75))
              .andThen(new InstantCommand(() -> setIndexerPercentSpeedForShooting(0.9)))
              .andThen(new WaitCommand(0.75))
              .finallyDo(() -> {stopIndexer(); stopMainRoller();});
    return command;
  }

  public Command shoot2ForAmpCommand(){
    Command command =
          new InstantCommand(() -> {setMainRollerPercentSpeedForShooting(0.4); setIndexerPercentSpeedForShooting(0.6);})
              .andThen(new WaitCommand(1.5))
              .finallyDo(() -> {stopIndexer(); stopMainRoller();});
    return command;
  }

  /**
   * Create a command to run intake until the beam break sensor is trigger, or it times out.
   * @param beamBreakSensor - the beam break sensor at the back of the shooter
   * @param timeOut - stop if beam is not broken in this time (seconds)
   * @return a command to do the intake
   */
  public Command intake2UntilBeamBreak(CollectorRoller collectorRoller, BeamBreakSensor beamBreakSensor){
    double timeOut = 5.0;
    Command command =
      new InstantCommand(() -> collectorRoller.pullIn())
          .andThen(new InstantCommand(() -> setIndexerPercentSpeedForIntake(0.3)))
          .andThen(new InstantCommand(() -> setMainRollerPercentSpeedForIntake(0.3)))
          .andThen(new InstantCommand(() -> System.out.println("Intake2UntilBeamBreal: waiting for noteIsInShooter")))
          .andThen(new WaitUntilCommand(() -> beamBreakSensor.noteIsInShooter()))
          .andThen(new InstantCommand(() -> System.out.println("   note is in shooter")))
          .withTimeout(timeOut)
          .finallyDo(() -> { stopIndexer(); stopMainRoller(); collectorRoller.stop();})
          ;
    return command;
  }
}
