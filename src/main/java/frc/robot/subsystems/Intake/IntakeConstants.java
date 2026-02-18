package frc.robot.subsystems.Intake;

public class IntakeConstants {
    public static final int IntakeMotorID = 19;
    public static final int ControlMotorID = 16;
    
    // INTAKE MOTOR //
    public static final double Intake_kS = 0;
    public static final double Intake_kV = 0;
    public static final double Intake_kA = 0;
    public static final double Intake_kP = 0;
    public static final double Intake_kI = 0;
    public static final double Intake_kD = 0;

    public static final double Intake_StatorCurrentLimit = 30;
    public static final double Intake_SupplyCurrentLimit = 30;

    public static final double INTAKE_POWER_STOP = 0;
    public static final double INTAKE_POWER_INTAKE = -0.7;
    public static final double INTAKE_POWER_DEPLETE = 0;

    // CONTROL MOTOR //
    public static final double Control_kS = 0;
    public static final double Control_kV = 0;
    public static final double Control_kA = 0;
    public static final double Control_kP = 0;
    public static final double Control_kI = 0;
    public static final double Control_kD = 0;

    public static final double Control_Expo_CV = 0;
    public static final double Control_Expo_kV = 0;
    public static final double Control_Expo_kA = 0;

    public static final double Control_StatorCurrentLimit = 30;
    public static final double Control_SupplyCurrentLimit = 30;

    public static final double CONTROL_POS_IN = 0;
    public static final double CONTROL_POS_OUT = 0;

    public static final double CONTROL_POS_TOLERANCE = 0;

    // OTHER //
    public static final double SimulationMotorGearRatio = 12.8;

    // SysId //
    public static final double Intake_SysIdQuasistatic = 0.5;
    public static final double Intake_SysIdDynamic = 2;
    public static final double Control_SysIdQuasistatic = 0.5;
    public static final double Control_SysIdDynamic = 2;
}
