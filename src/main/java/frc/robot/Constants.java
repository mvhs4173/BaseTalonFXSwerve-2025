package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
//import com.revrobotics.SparkRelativeEncoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.1;

    public static final class Swerve {
        public static final int pigeonID = 1;

        public static final COTSTalonFXSwerveConstants chosenModule =  //TODO: This must be tuned to specific robot
        COTSTalonFXSwerveConstants.SDS.MK4i.Falcon500(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L2);

        /* Drivetrain Constants */
        public static final double trackWidth = 52.5; //TODO: This must be tuned to specific robot
        public static final double wheelBase = 73.0; //TODO: This must be tuned to specific robot
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 25;
        public static final int angleCurrentThreshold = 40;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 40;
        public static final int driveCurrentThreshold = 60;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.2;
        public static final double closedLoopRamp = 0.2;
        /*Let's allow ramp control on angle motors as well.
         * "Ramp" is number of seconds between stopped and full speed */
        public static final double angleOpenLoopRamp = 0.2;
        public static final double angleClosedLoopRamp = .2;
        // Increasing closedLoopRamp lowers peak current draw,
        // but causes more oscillation in angle.

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP * 0.6;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD * 2.3;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.12; //TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = 0.32; //TODO: This must be tuned to specific robot
        public static final double driveKV = 1.51;
        public static final double driveKA = 0.27;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 3.5; //TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double maxAngularVelocity = 0.5; //TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 31;
            public static final int angleMotorID = 32;
            public static final int canCoderID = 41;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(-0.499);// Rotation2d.fromDegrees(-115.58);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 33;
            public static final int angleMotorID = 34;
            public static final int canCoderID = 43;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(0.961); //Rotation2d.fromDegrees(-30.85);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 38;
            public static final int angleMotorID = 37;
            public static final int canCoderID = 47;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(-0.968); //Rotation2d.fromDegrees(-126.56);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 36;
            public static final int angleMotorID = 35;
            public static final int canCoderID = 45;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(0.982); //Rotation2d.fromDegrees(+168.93);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 2.0;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1.0;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
    /**
     * Info regarding the two shooter motors: REV Neo's running in opposite directions.
     */
    public static final class ShooterConstants{
        public static final MotorType kMotorType = MotorType.kBrushless;
        public static final double encoderRotationsPerFinalRotation = 1.0;
        public static final class Left {
            public static final int kCANId = 11;
            public static final String kName = "Left Shooter";
        }
        public static final class Right {
            public static final int kCANId = 53;
            public static final String kName = "Right Shooter";
        }
        public static final class PID {
            // velocity PID values
            public static final double kP = 0.000100;
            public static final double kI = 0.000000;
            public static final double kD = 0.000000;
            public static final double kFeedForward = 0.000170 * 1.07;
            public static final double kIZone = 0.000000;
            public static final double kMinOutput = -1.0;
            public static final double kMaxOutput = +1.0;
        }
    }
    /**
     * Info regarding the motors powering the "shoulder", the joint between the chassis and the arm.
     * There are two motors, one at each end of a shaft, so one will be a follower of the other, in inverted mode.
     * They are currently REV Neo's with 140:1 planetary gearboxes.
     */
    public static final class ShoulderConstants {
        public static final MotorType kMotorType = MotorType.kBrushless;
        public static final double encoderRotationsPerFinalRotation = 140.0;
        public static final double lowerSoftLimit = -0.222; // in revolutions of shoulder
        public static final double upperSoftLimit = 0.01;
        public static final class Left{
            public static final int kCANId = 52;
            public static final String kName = "Left Shoulder";
        }
        public static final class Right{
            public static final int kCANId = 62;
            public static final String kName = "Right Shoulder";
        }
        public static final class PID {
            // velocity PID values
            public static final double kP = 0.000080; // TODO: fill in PID coefficients
            public static final double kI = 0.000000;
            public static final double kD = 0.000000;
            public static final double kFeedForward = 0.000170 * 1;
            public static final double kIZone = 0.000000;
            public static final double kMinOutput = -1.0;
            public static final double kMaxOutput = +1.0;
        }
    }
    /**
     * The wrist motor is a brushed 'seat motor' with a REV through bore
     * encoder attached to its shaft.  Do the motor and encoder agree on
     * which direction is positive?  Probably not.
     */
    public static class WristConstants{
        public static final int kCANId = 59;
        public static final MotorType kMotorType = MotorType.kBrushed;
        public static final double encoderRotationsPerFinalRotation = 1.0;
        public static final String kName = "Wrist";
        //public static final SparkRelativeEncoder.Type encoderType = SparkRelativeEncoder.Type.kQuadrature;
        
        public static int encoderCountsPerRevolution = 8192;
        public static double lowerSoftLimit = 0.0; // in rotations of wrist
        public static double upperSoftLimit = 0.28;

        public static final class PID {
            // velocity PID values
            public static final double kP = 0.000080; // TODO: fill in PID coefficients
            public static final double kI = 0.000000;
            public static final double kD = 0.000000;
            public static final double kFeedForward = 0.000000;
            public static final double kIZone = 0.000000;
            public static final double kMinOutput = -1.0;
            public static final double kMaxOutput = +1.0;
        }
        // positions are in rotations counterclockwise from zero
        // when looked at from robot's right.
        public static final double minSafePosition = 0.1;
        public static final double maxSafePosition = 0.2;
    }

    /**Collector roller is a brushless motor connected to a roller 
     * on the front of robot for intaking notes
     */
    public static class CollectorRollerConstants{
        public static final int kCANId = 55;
        public static final MotorType kMotorType = MotorType.kBrushless;
        public static final double encoderRotationsPerFinalRotation = 10.0;
        public static final String kName = "CollectorRoller";
        public static final double defaultPullInSpeed = 0.8; //percent speed
        public static final double defaultPushOutSpeed = 0.4; //percent speed
    }

    public static class BeamBreakSensorConstants{
        public static final int channel = 0;
        public static final boolean normallyOpen = true;
        public static final String name = "BeamBreak";
    }

    public static class Shooter2Constants{
        public static class UpperRoller{ // a NEO
            public static final int kCANId = 48; // was 41 (dup of fr cancoder) until 2024-12-10
            public static final MotorType kMotorType = MotorType.kBrushless;
            public static final double encoderRotationsPerFinalRotation = 1.0;
            public static final String name = "Upper Shooter2 Roller";
        }
        public static class LowerRoller{ // a NEO
            public static final int kCANId = 40;
            public static final MotorType kMotorType = MotorType.kBrushless;
            public static final double encoderRotationsPerFinalRotation = 1.0;
            public static final String name = "Lower Shooter2 Roller";
        }
        public static class Indexer{ // a NEO 550
            public static final int kCANId = 42;
            public static final MotorType kMotorType = MotorType.kBrushless;
            public static final double encoderRotationsPerFinalRotation = 10.0 / 1.0 // planetary gearbox
                                                                        * 22.0 / 48.0; // sprockets
            public static final String name = "Shooter2 Indexer";
        }
    }
    public static class Wrist2Constants{
        public static final int kCANId = 12; // TODO : get the can id from programmers' notebook
        public static final double encoderRotationsPerFinalRotation = 7.0 * 7.0 // planetary gearbox
                                                                    * 48.0 / 16.0; // sprockets
        public static final String name = "Wrist2";
        public static double lowerSoftLimit = 0.0; // in rotations of wrist
        public static double upperSoftLimit = 0.28; // TODO: check this empirically
    }
}
