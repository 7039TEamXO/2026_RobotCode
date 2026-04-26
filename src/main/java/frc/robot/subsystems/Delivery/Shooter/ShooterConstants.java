package frc.robot.subsystems.Delivery.Shooter;

public class ShooterConstants {
    public static final int ShooterEntryMotorID = 16;
    public static final int ShooterAngleMotorID = 11;
    public static final int ShooterMasterMotorID = 19;
    public static final int ShooterSlaveMotorID = 5;

    public static final int AbsoluteEncoderID = 0;

    public static final double EncoderGearRatio = 1/6.0;
    public static final double EncoderOffset = 0.0;

    // SHOOTER MOTOR(S) //

    public static final double Shoot_StatorCurrentLimit = 120;
    public static final double Shoot_SupplyCurrentLimit = 100;

    public static final double Shoot_kS = 0.13515;
    public static final double Shoot_kV = 0.1163;
    public static final double Shoot_kA = 0.010804;
    public static final double Shoot_kP = 0.069845;
    public static final double Shoot_kI = 0;
    public static final double Shoot_kD = 0;

    public static final double SHOOT_VELOCITY_IDLE = 5;
    public static final double SHOOT_VELOCITY_HELP = 7;
    public static final double SHOOT_VELOCITY_TOLERANCE = 4;

    public static final double SHOOT_VELOCITY_MIN = 0; // Disabled

    // ENTRY MOTOR //

    public static final double Entry_kS = 0.37413;
    public static final double Entry_kV = 0.12912;
    public static final double Entry_kA = 0.010361;
    public static final double Entry_kP = 0.077853;
    public static final double Entry_kI = 0;
    public static final double Entry_kD = 0;

    public static final double Entry_StatorCurrentLimit = 90;
    public static final double Entry_SupplyCurrentLimit = 70;


    public static final double ENTRY_VELOCITY_STOP = 0;
    public static final double ENTRY_VELOCITY_ROLL = 60;
    public static final double ENTRY_VELOCITY_REVERSE = -60;

    public static final double ENTRY_VELOCITY_MIN = 0; // Disabled

    // ANGLE MOTOR //
    
    public static final double Angle_kS = 0.15; // 0.15
    public static final double Angle_kV = 0; // 0.15
    public static final double Angle_kA = 0; // 0.05
    public static final double Angle_kP = 3.5; // 1
    public static final double Angle_kI = 0.02;
    public static final double Angle_kD = 0;

    public static final double Angle_CruiseVelocity = 80;
    public static final double Angle_Acceleration = 130;
    public static final double Angle_Jerk = 1600;

    public static final double Angle_StatorCurrentLimit = 60;
    public static final double Angle_SupplyCurrentLimit = 40;

    public static final double ANGLE_INTAKE_HELP_POS = 5; // Rotations
    public static final double ANGLE_NEUTRAL_ZONE_POS = 7; // Rotations

    // OTHER //

    public static final double SimulationMotorGearRatio = 12.8;

    // SysId //

    public static final double Shoot_SysIdQuasistatic = 1;
    public static final double Shoot_SysIdDynamic = 2;

    public static final double Angle_SysIdQuasistatic = 0.75;
    public static final double Angle_SysIdDynamic = 1.5;

    public static final double Entry_SysIdQuasistatic = 1;
    public static final double Entry_SysIdDynamic = 2;
    
    // Shooter mapping //

    public static final double ANGLE_SHOOT_MAX = 6.0;
    public static final double RPS_SHOOT_MAX = 65;
    public static final double TIME_SHOOT_MAX = 3;

    public static final double RPS_DELIVERY_CLOSE_RAMP = 20;
    public static final double RPS_DELIVERY_FAR_RAMP = 40;
    public static final double RPS_DELIVERY_OPPONENT_SIDE = 85;
}
