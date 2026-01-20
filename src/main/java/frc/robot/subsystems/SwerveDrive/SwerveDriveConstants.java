package frc.robot.subsystems.SwerveDrive;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;

import com.therekrab.autopilot.APConstraints;
import com.therekrab.autopilot.APProfile;
import com.therekrab.autopilot.Autopilot;

import edu.wpi.first.math.util.Units;

               
public class SwerveDriveConstants {
    // SPEED LIMITATIONS
    public static final double MAX_SPEED = 4.5; // NOT TUNEABLE [!]
    public static final double MAX_ACCEL = 2.5;

    public static final double MIN_SPEED  = Units.feetToMeters(4); // Non-restrictive
    public static final double CLIMB_SPEED = Units.feetToMeters(7);

    public static final double MAX_ROTATION_V = 4;

    // SysId
    public static final double DriveSysIdQuasistatic = 1;
    public static final double DriveSysIdDynamic = 7;
    public static final double TurnSysIdQuasistatic = 1;
    public static final double TurnSysIdDynamic = 7;

    // Angular motion PIDs
    public static final double KP_ANGULAR = 3;
    public static final double KD_ANGULAR = 0;

    // Autopilot
    private static final APConstraints kConstraints = new APConstraints()
        .withAcceleration(7039.0) // Temporary?
        .withJerk(2.0); // Simulation-induced

    private static final APProfile kProfile = new APProfile(kConstraints)
        .withErrorXY(Centimeters.of(2))
        .withErrorTheta(Degrees.of(0.5))
        .withBeelineRadius(Centimeters.of(8));

    public static final Autopilot kAutopilot = new Autopilot(kProfile);

    // PROXIMITY TOLERANCE
    public static final double CLOSE_DISTANCE_ERROR_CAP = 0.3;
    public static final double CLOSE_ANGLE_ERROR_CAP = Math.toRadians(3);

    // ERROR TOLERANCE
    public static final double MAXIMUM_DISTANCE_ERROR = 0.5;
    // public static final double MAXIMUM_ANGLE_ERROR = 10;
}