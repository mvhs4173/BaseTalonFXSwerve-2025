package frc.robot.autos;

import frc.robot.commands.positionCommands.goToCollectionPositionFromAmp;
import frc.robot.commands.positionCommands.goToSpeakerShotPosition;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Wrist2;
import frc.robot.subsystems.Shooter2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class centerSpeaker extends SequentialCommandGroup {
    Swerve s_Swerve;
    Shoulder m_shoulder;
    Wrist2 m_wrist2;
    Shooter2 m_shooter2;
    public centerSpeaker(Swerve swerve, Shoulder shoulder, Wrist2 wrist2, Shooter2 shooter2){
        s_Swerve = swerve;
        m_shoulder = shoulder;
        m_wrist2 = wrist2;
        m_shooter2 = shooter2;
        addRequirements(s_Swerve, m_shoulder, m_wrist2, m_shooter2);

        addCommands(
            new InstantCommand(() -> m_wrist2.setToBrakeOnIdle(true)),
            new InstantCommand(() -> s_Swerve.setPose(new Pose2d(Units.feetToMeters(0), Units.feetToMeters(0), new Rotation2d(Units.degreesToRadians(180))))),
            new InstantCommand(() -> s_Swerve.lockX()),
            new WaitCommand(0.25),
            new goToSpeakerShotPosition(m_shoulder, m_wrist2),
            new WaitCommand(0.25),
            m_shooter2.shoot2ForSpeakerCommand(),
            new WaitCommand(0.25),
            new goToCollectionPositionFromAmp(shoulder, wrist2)
        );
    }
}