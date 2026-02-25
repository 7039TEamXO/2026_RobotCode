package frc.robot.subsystems.SwerveDrive;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import frc.robot.Constants;
import frc.robot.Constants.TuningMode;

public class SwerveDriveTuning {
    private static final LoggedNetworkNumber JOYSTICK_SLOWDOWN_FACTOR = new LoggedNetworkNumber("/Tuning/Swerve/Joystick Slowdown Factor", SwerveDriveConstants.JOYSTICK_SLOW_DOWN_FACTOR);
    private static final LoggedNetworkNumber JOYSTICK_SLOWDOWN_FACTOR_SHOOT = new LoggedNetworkNumber("/Tuning/Swerve/Joystick Slowdown Factor Shoot", SwerveDriveConstants.JOYSTICK_SLOW_DOWN_FACTOR_SHOOT);

    private static final LoggedNetworkNumber KP = new LoggedNetworkNumber("/Tuning/Swerve/Drive kP", SwerveDriveConstants.KP);
    private static final LoggedNetworkNumber KD = new LoggedNetworkNumber("/Tuning/Swerve/Drive kD", SwerveDriveConstants.KD);

    private static final LoggedNetworkNumber KP_ANGULAR = new LoggedNetworkNumber("/Tuning/Swerve/(DTP) Angle kP", SwerveDriveConstants.KP_ANGULAR);
    private static final LoggedNetworkNumber KD_ANGULAR = new LoggedNetworkNumber("/Tuning/Swerve/(DTP) Angle kD", SwerveDriveConstants.KD_ANGULAR);
    private static final LoggedNetworkNumber KP_ANGULAR_SLICK = new LoggedNetworkNumber("/Tuning/Swerve/(DTP) Angle kP Slick", SwerveDriveConstants.KP_ANGULAR_SLICK);
    private static final LoggedNetworkNumber KD_ANGULAR_SLICK = new LoggedNetworkNumber("/Tuning/Swerve/(DTP) Angle kD Slick", SwerveDriveConstants.KD_ANGULAR_SLICK);

    private static final LoggedNetworkNumber CLOSE_DISTANCE_ERROR_CAP = new LoggedNetworkNumber("/Tuning/Swerve/Close Distance Error Cap", SwerveDriveConstants.CLOSE_DISTANCE_ERROR_CAP);
    private static final LoggedNetworkNumber CLOSE_ANGLE_ERROR_CAP = new LoggedNetworkNumber("/Tuning/Swerve/Close Angle Error Cap", SwerveDriveConstants.CLOSE_ANGLE_ERROR_CAP);
    private static final LoggedNetworkNumber PRECISE_ANGLE_ERROR_CAP = new LoggedNetworkNumber("/Tuning/Swerve/Precise Angle Error Cap", SwerveDriveConstants.PRECISE_ANGLE_ERROR_CAP);

    private static final LoggedNetworkNumber TEST_DRIVE_X = new LoggedNetworkNumber("/Tuning/Swerve/(Test L3) X", SwerveDriveConstants.TEST_DRIVE_X);
    private static final LoggedNetworkNumber TEST_DRIVE_Y = new LoggedNetworkNumber("/Tuning/Swerve/(Test L3) Y", SwerveDriveConstants.TEST_DRIVE_Y);
    private static final LoggedNetworkNumber TEST_DRIVE_ANGLE = new LoggedNetworkNumber("/Tuning/Swerve/(Test L3) Angle", SwerveDriveConstants.TEST_DRIVE_ANGLE);
    private static final LoggedNetworkNumber VX_VELOCITY_THRESHOLD = new LoggedNetworkNumber("/Tuning/Swerve/VX Velocity Threshold", SwerveDriveConstants.VX_VELOCITY_THRESHOLD);
    private static final LoggedNetworkNumber VY_VELOCITY_THRESHOLD = new LoggedNetworkNumber("/Tuning/Swerve/VY Velocity Threshold", SwerveDriveConstants.VY_VELOCITY_THRESHOLD);
    private static final LoggedNetworkNumber OMEGA_VELOCITY_THRESHOLD = new LoggedNetworkNumber("/Tuning/Swerve/OMEGA Velocity Threshold", SwerveDriveConstants.OMEGA_VELOCITY_THRESHOLD);
    private static final LoggedNetworkNumber HUB_VELOCITY_AWARE_BLEND = new LoggedNetworkNumber("/Tuning/Swerve/Hub Velocity Aware Blend", SwerveDriveConstants.HUB_VELOCITY_AWARE_BLEND);

    public static final double JOYSTICK_SLOWDOWN_FACTOR_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? JOYSTICK_SLOWDOWN_FACTOR.get() : SwerveDriveConstants.JOYSTICK_SLOW_DOWN_FACTOR;
    }

    public static final double JOYSTICK_SLOWDOWN_FACTOR_SHOOT_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? JOYSTICK_SLOWDOWN_FACTOR_SHOOT.get() : SwerveDriveConstants.JOYSTICK_SLOW_DOWN_FACTOR_SHOOT;
    }

    public static final double KP_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? KP.get() : SwerveDriveConstants.KP;
    }

    public static final double KD_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? KD.get() : SwerveDriveConstants.KD;
    }

    public static final double KP_ANGULAR_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? KP_ANGULAR.get() : SwerveDriveConstants.KP_ANGULAR;
    }

    public static final double KD_ANGULAR_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? KD_ANGULAR.get() : SwerveDriveConstants.KD_ANGULAR;
    }

    public static final double KP_ANGULAR_SLICK_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? KP_ANGULAR_SLICK.get() : SwerveDriveConstants.KP_ANGULAR_SLICK;
    }

    public static final double KD_ANGULAR_SLICK_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? KD_ANGULAR_SLICK.get() : SwerveDriveConstants.KD_ANGULAR_SLICK;
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

    public static final double VX_VELOCITY_THRESHOLD_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? VX_VELOCITY_THRESHOLD.get() : SwerveDriveConstants.VX_VELOCITY_THRESHOLD;
    }

    public static final double VY_VELOCITY_THRESHOLD_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? VY_VELOCITY_THRESHOLD.get() : SwerveDriveConstants.VY_VELOCITY_THRESHOLD;
    }

    public static final double OMEGA_VELOCITY_THRESHOLD_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? OMEGA_VELOCITY_THRESHOLD.get() : SwerveDriveConstants.OMEGA_VELOCITY_THRESHOLD;
    }

    /** 0 = position-only hub angle, 1 = full velocity-aware hub angle. */
    public static final double HUB_VELOCITY_AWARE_BLEND_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? HUB_VELOCITY_AWARE_BLEND.get() : SwerveDriveConstants.HUB_VELOCITY_AWARE_BLEND;
    }

    // Activates on an POV = 90 press
    public static final double TEST_DRIVE_X_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? TEST_DRIVE_X.get() : SwerveDriveConstants.TEST_DRIVE_X;
    }

    public static final double TEST_DRIVE_Y_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? TEST_DRIVE_Y.get() : SwerveDriveConstants.TEST_DRIVE_Y;
    }

    public static final double TEST_DRIVE_ANGLE_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? TEST_DRIVE_ANGLE.get() : SwerveDriveConstants.TEST_DRIVE_ANGLE;
    }
}
