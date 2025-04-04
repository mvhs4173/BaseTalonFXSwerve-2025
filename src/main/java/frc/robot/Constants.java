package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.1;

    public static final PneumaticsModuleType pneumaticsModuleType = PneumaticsModuleType.REVPH;

    public static final class Swerve {
        public static final COTSTalonFXSwerveConstants chosenModule =  //This must be tuned to specific robot
          COTSTalonFXSwerveConstants.SDS.MK4i.Falcon500(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L2);

        /* Drivetrain Constants */
        public static final double trackWidth = 52.5; //This must be tuned to specific robot
        public static final double wheelBase = 73.0; //This must be tuned to specific robot
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
        public static final double angleKP = 50;
        public static final double angleKI = 0;
        public static final double angleKD = 0.5;
        public static final double angleKS = 0.18;
        public static final double angleKV = 2.66;
        public static final double angleKA = 0.018;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.12; //This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = 0.32; //This must be tuned to specific robot
        public static final double driveKV = 1.51;
        public static final double driveKA = 0.27;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 3.5; //This must be tuned to specific robot
        /** Radians per Second */
        public static final double maxAngularVelocity = 0.5; //This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { //This must be tuned to specific robot
            public static final CANId driveMotorID = CANId.FRONT_LEFT_DRIVE;
            public static final CANId angleMotorID = CANId.FRONT_LEFT_STEER;
            public static final CANId canCoderID = CANId.FRONT_LEFT_CANCODER;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(0.494);// Rotation2d.fromDegrees(-115.58);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { //This must be tuned to specific robot
            public static final CANId driveMotorID = CANId.FRONT_RIGHT_DRIVE;
            public static final CANId angleMotorID = CANId.FRONT_RIGHT_STEER;
            public static final CANId canCoderID = CANId.FRONT_RIGHT_CANCODER;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(-0.018); //Rotation2d.fromDegrees(-30.85);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { //This must be tuned to specific robot
            public static final CANId driveMotorID = CANId.BACK_LEFT_DRIVE;
            public static final CANId angleMotorID = CANId.BACK_LEFT_STEER;
            public static final CANId canCoderID = CANId.BACK_LEFT_CANCODER;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(-0.007); //Rotation2d.fromDegrees(-126.56);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { //This must be tuned to specific robot
            public static final CANId driveMotorID = CANId.BACK_RIGHT_DRIVE;
            public static final CANId angleMotorID = CANId.BACK_RIGHT_STEER;
            public static final CANId canCoderID = CANId.BACK_RIGHT_CANCODER;
            public static final Rotation2d angleOffset = Rotation2d.fromRotations(-0.01); //Rotation2d.fromDegrees(+168.93);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final class AutoConstants { //The below constants are used in the example auto, and must be tuned to specific robot
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
}
