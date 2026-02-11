package frc.robot.subsystems.Delivery.Shooter;

public class ShooterConstants {
    public static final int ShooterEntryMotorID = -1;
    public static final int ShooterAngleMotorID = -1;
    public static final int ShooterMasterMotorID = -1;
    public static final int ShooterSlaveMotorID = -1;

    public static final int AbsoluteEncoderID = -1;

    public static final double EncoderGearRatio = 1/6.0;
    public static final double EncoderOffset = 0.0;

    // ENTRY MOTOR //

    public static final double Entry_kS = 0;
    public static final double Entry_kV = 0;
    public static final double Entry_kA = 0;
    public static final double Entry_kP = 0;
    public static final double Entry_kI = 0;
    public static final double Entry_kD = 0;

    public static final double Entry_StatorCurrentLimit = 30;
    public static final double Entry_SupplyCurrentLimit = 30;

    public static final double ENTRY_POWER_STOP = 0;
    public static final double ENTRY_POWER_ROLL = 0;

    // ANGLE MOTOR //
    
    public static final double Angle_kS = 0;
    public static final double Angle_kV = 0;
    public static final double Angle_kA = 0;
    public static final double Angle_kP = 0;
    public static final double Angle_kI = 0;
    public static final double Angle_kD = 0;

    public static final double Angle_Expo_CV = 0;
    public static final double Angle_Expo_kV = 0;
    public static final double Angle_Expo_kA = 0;

    public static final double Angle_StatorCurrentLimit = 30;
    public static final double Angle_SupplyCurrentLimit = 30;

    // SHOOT MOTOR //

    public static final double Shoot_kS = 0;
    public static final double Shoot_kV = 0;
    public static final double Shoot_kA = 0;
    public static final double Shoot_kP = 0;
    public static final double Shoot_kI = 0;
    public static final double Shoot_kD = 0;

    public static final double Shoot_StatorCurrentLimit = 50;
    public static final double Shoot_SupplyCurrentLimit = 50;

    public static final double SHOOT_VOLTAGE_IDLE = 0;
    public static final double SHOOT_VOLTAGE_FIRE = 0;
    public static final double SHOOT_VOLTAGE_TOLERANCE = 0;

    // OTHER //

    public static final double SimulationMotorGearRatio = 12.8;

    // SysId //

    public static final double Entry_SysIdQuasistatic = 0.5;
    public static final double Entry_SysIdDynamic = 2;

    public static final double Angle_SysIdQuasistatic = 0.5;
    public static final double Angle_SysIdDynamic = 2;
    
    public static final double Shoot_SysIdQuasistatic = 0.5;
    public static final double Shoot_SysIdDynamic = 2;
}
