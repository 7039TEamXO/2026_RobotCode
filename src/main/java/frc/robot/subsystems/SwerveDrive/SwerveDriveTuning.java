package frc.robot.subsystems.SwerveDrive;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import frc.robot.Constants;
import frc.robot.Constants.TuningMode;

public class SwerveDriveTuning {
    private static final LoggedNetworkNumber MIN_SPEED = new LoggedNetworkNumber("/Tuning/Swerve/Minimum Speed", SwerveDriveConstants.MIN_SPEED);
    private static final LoggedNetworkNumber CLIMB_SPEED = new LoggedNetworkNumber("/Tuning/Swerve/Climb Speed", SwerveDriveConstants.CLIMB_SPEED);
    private static final LoggedNetworkNumber MAX_ROTATION_V = new LoggedNetworkNumber("/Tuning/Swerve/Maximum Rotation Velocity", SwerveDriveConstants.MAX_ROTATION_V);

    private static final LoggedNetworkNumber KP_ANGULAR = new LoggedNetworkNumber("/Tuning/Swerve/(DTP) Angle kP", SwerveDriveConstants.KP_ANGULAR);
    private static final LoggedNetworkNumber KD_ANGULAR = new LoggedNetworkNumber("/Tuning/Swerve/(DTP) Angle kD", SwerveDriveConstants.KD_ANGULAR);

    private static final LoggedNetworkNumber CLOSE_DISTANCE_ERROR_CAP = new LoggedNetworkNumber("/Tuning/Swerve/Close Distance Error Cap", SwerveDriveConstants.CLOSE_DISTANCE_ERROR_CAP);
    private static final LoggedNetworkNumber CLOSE_ANGLE_ERROR_CAP = new LoggedNetworkNumber("/Tuning/Swerve/Close Angle Error Cap", SwerveDriveConstants.CLOSE_ANGLE_ERROR_CAP);
    private static final LoggedNetworkNumber PRECISE_ANGLE_ERROR_CAP = new LoggedNetworkNumber("/Tuning/Swerve/Precise Angle Error Cap", SwerveDriveConstants.PRECISE_ANGLE_ERROR_CAP);

    private static final LoggedNetworkNumber TEST_DRIVE_X = new LoggedNetworkNumber("/Tuning/Swerve/(Test L3) X", 0);
    private static final LoggedNetworkNumber TEST_DRIVE_Y = new LoggedNetworkNumber("/Tuning/Swerve/(Test L3) Y", 0);
    private static final LoggedNetworkNumber TEST_DRIVE_ANGLE = new LoggedNetworkNumber("/Tuning/Swerve/(Test L3) Angle", 0);

    public static final double MIN_SPEED_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? MIN_SPEED.get() : SwerveDriveConstants.MIN_SPEED;
    }

    public static final double CLIMB_SPEED_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? CLIMB_SPEED.get() : SwerveDriveConstants.CLIMB_SPEED;
    }

    public static final double MAX_ROTATION_V_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? MAX_ROTATION_V.get() : SwerveDriveConstants.MAX_ROTATION_V;
    }

    public static final double KP_ANGULAR_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? KP_ANGULAR.get() : SwerveDriveConstants.KP_ANGULAR;
    }

    public static final double KD_ANGULAR_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? KD_ANGULAR.get() : SwerveDriveConstants.KD_ANGULAR;
    }

    public static final double CLOSE_DISTANCE_ERROR_CAP_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? CLOSE_DISTANCE_ERROR_CAP.get() : SwerveDriveConstants.CLOSE_DISTANCE_ERROR_CAP;
    }

    public static final double CLOSE_ANGLE_ERROR_CAP_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? CLOSE_ANGLE_ERROR_CAP.get() : SwerveDriveConstants.CLOSE_ANGLE_ERROR_CAP;
    }

    public static final double PRECISE_ANGLE_ERROR_CAP_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? PRECISE_ANGLE_ERROR_CAP.get() : SwerveDriveConstants.PRECISE_ANGLE_ERROR_CAP;
    }

    // Activates on an R1 press
    public static final double TEST_DRIVE_X_get() {
        return TEST_DRIVE_X.get();
    }

    public static final double TEST_DRIVE_Y_get() {
        return TEST_DRIVE_Y.get();
    }

    public static final double TEST_DRIVE_ANGLE_get() {
        return TEST_DRIVE_ANGLE.get();
    }
}
