package frc.robot.subsystems.Delivery.Shooter;

public class ShooterConstants {
    public static final int ShooterEntryMotorID = 14;
    public static final int ShooterAngleMotorID = 13;
    public static final int ShooterMasterMotorID = 15;
    public static final int ShooterSlaveMotorID = 17;

    public static final int AbsoluteEncoderID = 0;

    public static final double EncoderGearRatio = 1/6.0;
    public static final double EncoderOffset = 0.0;

    // ENTRY MOTOR //

    public static final double Entry_kS = 0.15371;
    public static final double Entry_kV = 0.12269;
    public static final double Entry_kA = 0.032478;
    public static final double Entry_kP = 0.15806;
    public static final double Entry_kI = 0;
    public static final double Entry_kD = 0;

    public static final double Entry_StatorCurrentLimit = 40;
    public static final double Entry_SupplyCurrentLimit = 20;

    public static final double ENTRY_POWER_STOP = 0;
    public static final double ENTRY_POWER_ROLL = 1;

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

    public static final double Angle_StatorCurrentLimit = 120;
    public static final double Angle_SupplyCurrentLimit = 70;

    public static final double ANGLE_INTAKE_HELP_POS = 2; // Rotations

    // SHOOT MOTOR //

    public static final double Shoot_StatorCurrentLimit = 120;
    public static final double Shoot_SupplyCurrentLimit = 70;

    public static final double SHOOT_VOLTAGE_IDLE = 0.2;
    public static final double SHOOT_VOLTAGE_FIRE = 6;
    public static final double SHOOT_VOLTAGE_INTAKE = 1.5;

    public static final double SHOOT_RPM_FIRE = 0;
    public static final double SHOOT_RPM_TOLERANCE = 10;

    // OTHER //

    public static final double SimulationMotorGearRatio = 12.8;

    // SysId //
    public static final double Angle_SysIdQuasistatic = 0.75;
    public static final double Angle_SysIdDynamic = 1.5;
}
