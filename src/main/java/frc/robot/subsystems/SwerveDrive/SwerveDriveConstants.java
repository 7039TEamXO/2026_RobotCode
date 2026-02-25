package frc.robot.subsystems.SwerveDrive;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;

import com.therekrab.autopilot.APConstraints;
import com.therekrab.autopilot.APProfile;
import com.therekrab.autopilot.Autopilot;
               
public class SwerveDriveConstants {
    // SPEED LIMITATIONS
    public static final double MAX_SPEED = 4.5; // NOT TUNEABLE [!]

    public static final double JOYSTICK_SLOW_DOWN_FACTOR = 0.8;
    public static final double JOYSTICK_SLOW_DOWN_FACTOR_SHOOT = 0.2;

    // SysId
    public static final double DriveSysIdQuasistatic = 1;
    public static final double DriveSysIdDynamic = 7;
    public static final double TurnSysIdQuasistatic = 1;
    public static final double TurnSysIdDynamic = 7;

    // End drive PIDs
    public static final double KP = 2;
    public static final double KD = 0;

    // Angular motion PIDs
    public static final double KP_ANGULAR = 3;
    public static final double KD_ANGULAR = 0;

    public static final double KP_ANGULAR_SLICK = 5;
    public static final double KD_ANGULAR_SLICK = 0;

    // Autopilot
    private static final APConstraints kConstraints = new APConstraints()
        .withAcceleration(7039.0) // Temporary?
        .withJerk(2); // Simulation-induced

    private static final APProfile kProfile = new APProfile(kConstraints)
        .withErrorXY(Centimeters.of(20))
        .withErrorTheta(Degrees.of(1))
        .withBeelineRadius(Centimeters.of(10));

    public static final Autopilot kAutopilot = new Autopilot(kProfile);

    // PROXIMITY TOLERANCE
    public static final double CLOSE_DISTANCE_ERROR_CAP = 0.05;
    public static final double CLOSE_ANGLE_ERROR_CAP = 5;
    public static final double PRECISE_ANGLE_ERROR_CAP = 2;

    public static final double VX_VELOCITY_THRESHOLD = 0.45;
    public static final double VY_VELOCITY_THRESHOLD = 0.45;
    public static final double OMEGA_VELOCITY_THRESHOLD = 1.5;

    /** Blend for hub aim: 0 = position-only angle, 1 = full velocity-aware angle. */
    public static final double HUB_VELOCITY_AWARE_BLEND = 1.0;

    // TEST DTP
    public static final double TEST_DRIVE_X = 13.4;
    public static final double TEST_DRIVE_Y = 4;
    public static final double TEST_DRIVE_ANGLE = -180;
}