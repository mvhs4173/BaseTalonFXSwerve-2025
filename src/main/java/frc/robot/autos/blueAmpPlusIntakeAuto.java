package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.commands.positionCommands.goToAmpShotPosition;
import frc.robot.commands.positionCommands.goToCollectionPositionFromAmp;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Wrist2;
import frc.robot.subsystems.BeamBreakSensor;
import frc.robot.subsystems.CollectorRoller;
import frc.robot.subsystems.Shooter2;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class blueAmpPlusIntakeAuto extends SequentialCommandGroup {
    Swerve s_Swerve;
    Shoulder m_shoulder;
    Wrist2 m_wrist2;
    Shooter2 m_shooter2;
    CollectorRoller m_collectorRoller;
    BeamBreakSensor m_beamBreakSensor;
    public blueAmpPlusIntakeAuto(Swerve swerve, Shoulder shoulder, Wrist2 wrist2, Shooter2 shooter2, CollectorRoller collectorRoller, BeamBreakSensor beamBreakSensor){
        s_Swerve = swerve;
        m_shoulder = shoulder;
        m_wrist2 = wrist2;
        m_shooter2 = shooter2;
        m_collectorRoller = collectorRoller;
        m_beamBreakSensor = beamBreakSensor;
        addRequirements(s_Swerve, m_shoulder, m_wrist2, m_shooter2, m_collectorRoller, m_beamBreakSensor);

        TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.swerveKinematics);
        
        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);



        Trajectory firstPathTrajectory =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(Units.feetToMeters(0), Units.feetToMeters(3.0), new Rotation2d(Units.degreesToRadians(90))),
                //Must change 'Y' value by at least 0.02 somewhere in the sequence
                List.of(
                     new Translation2d(Units.feetToMeters(1.0), Units.feetToMeters(2.75))
                    ),
                new Pose2d(Units.feetToMeters(2.0), Units.feetToMeters(3.0), new Rotation2d(Units.degreesToRadians(90))),
                config);

        SwerveControllerCommand firstPathCommand =
            new SwerveControllerCommand(
                firstPathTrajectory,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);
        
         Trajectory secondPathTrajectory =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                s_Swerve.getPose(),
                //Must change 'Y' value by at least 0.02 somewhere in the sequence
                List.of(
                     new Translation2d(Units.feetToMeters(2.0), Units.feetToMeters(0.5))
                    ),
                new Pose2d(Units.feetToMeters(5.0), Units.feetToMeters(0.5), new Rotation2d(Units.degreesToRadians(0))),
                config);

        SwerveControllerCommand secondPathCommand =
            new SwerveControllerCommand(
                secondPathTrajectory,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);


        addCommands(
            new InstantCommand(() -> s_Swerve.setPose(firstPathTrajectory.getInitialPose())),
            firstPathCommand,
            new InstantCommand(() -> s_Swerve.lockX()),
            new WaitCommand(0.25),
            new goToAmpShotPosition(m_shoulder, m_wrist2),
            new WaitCommand(0.25),
            m_shooter2.shoot2ForAmpCommand(),
            new WaitCommand(0.25),
            new goToCollectionPositionFromAmp(shoulder, wrist2),
            new WaitCommand(0.25),
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    secondPathCommand,
                    new InstantCommand(() -> s_Swerve.lockX())
                ),
                new SequentialCommandGroup(
                    new WaitCommand(2.0),
                    m_shooter2.intake2UntilBeamBreak(m_collectorRoller, m_beamBreakSensor)
                )
            )
        );
    }
}