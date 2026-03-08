package frc.robot.subsystems.Delivery.Shooter;

public class ShooterConstants {
    public static final int ShooterEntryMotorID = 16;
    public static final int ShooterAngleMotorID = 11;
    public static final int ShooterMasterMotorID = 19;
    public static final int ShooterSlaveMotorID = 5;

    public static final int AbsoluteEncoderID = 0;

    public static final double EncoderGearRatio = 1/6.0;
    public static final double EncoderOffset = 0.0;

    // ENTRY MOTOR //
    public static final double Entry_StatorCurrentLimit = 90;
    public static final double Entry_SupplyCurrentLimit = 70;

    public static final double ENTRY_POWER_STOP = 0;
    public static final double ENTRY_POWER_ROLL = 1;
    public static final double ENTRY_POWER_REVERSE = -0.5;

    // ANGLE MOTOR //
    
    public static final double Angle_kS = 0.5; // 0.15
    public static final double Angle_kV = 0; // 0.15
    public static final double Angle_kA = 0; // 0.05
    public static final double Angle_kP = 4; // 1
    public static final double Angle_kI = 0;
    public static final double Angle_kD = 0;

    public static final double Angle_CruiseVelocity = 160;
    public static final double Angle_Acceleration = 400;
    public static final double Angle_Jerk = 1600;

    public static final double Angle_StatorCurrentLimit = 60;
    public static final double Angle_SupplyCurrentLimit = 40;

    public static final double ANGLE_INTAKE_HELP_POS = 3; // Rotations
    public static final double ANGLE_NEUTRAL_ZONE_POS = 5; // Rotations

    public static final double Shoot_StatorCurrentLimit = 120;
    public static final double Shoot_SupplyCurrentLimit = 70;

    public static final double Shoot_kS = 0.14302;
    public static final double Shoot_kV = 0.12465;
    public static final double Shoot_kA = 0.005227;
    public static final double Shoot_kP = 0.067;
    public static final double Shoot_kI = 0;
    public static final double Shoot_kD = 0;

    public static final double SHOOT_VELOCITY_IDLE = 5;
    public static final double SHOOT_VELOCITY_HELP = 9;
    public static final double SHOOT_VELOCITY_NEUTRAL_ZONE = 60;
    public static final double SHOOT_VELOCITY_TOLERANCE = 4;

    // OTHER //

    public static final double SimulationMotorGearRatio = 12.8;

    // SysId //

    public static final double Angle_SysIdQuasistatic = 0.75;
    public static final double Angle_SysIdDynamic = 1.5;
    
    // Shooter mapping //
    
    public static final double HUB_SHOOT_DISTANCE = 1.0;
    public static final double CLIMB_SHOOT_DISTANCE = 3.0;
    public static final double CORNER_SHOOT_DISTANCE = 5.0;

    public static final double ANGLE_SHOOT_MAX = 5.0;
    
    public static final double RPS_SHOOT_MAX = 100;
}
