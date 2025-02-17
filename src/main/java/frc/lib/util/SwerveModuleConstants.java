package frc.lib.util;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.CANId;

public class SwerveModuleConstants {
    public final CANId driveMotorID;
    public final CANId angleMotorID;
    public final CANId cancoderID;
    public final Rotation2d angleOffset;

    /**
     * Swerve Module Constants to be used when creating swerve modules.
     * @param driveMotorID
     * @param angleMotorID
     * @param canCoderID
     * @param angleOffset
     */
    public SwerveModuleConstants(CANId driveMotorID, CANId angleMotorID, CANId canCoderID, Rotation2d angleOffset) {
        this.driveMotorID = driveMotorID;
        this.angleMotorID = angleMotorID;
        this.cancoderID = canCoderID;
        this.angleOffset = angleOffset;
    }
}
