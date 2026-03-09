package frc.robot.subsystems.Intake;

public class IntakeConstants {
    public static final int IntakeMotorID = 14;
    public static final int ControlMotorID = 12;
    
    // INTAKE MOTOR //
    public static final double Intake_StatorCurrentLimit = 80;
    public static final double Intake_SupplyCurrentLimit = 60;

    public static final double INTAKE_POWER_STOP = 0;
    public static final double INTAKE_POWER_INTAKE = -0.8;
    public static final double INTAKE_POWER_HELP_SHOOT = -0.6;
    public static final double INTAKE_POWER_DEPLETE = 0.8;

    // CONTROL MOTOR //
    public static final double CONTROL_POWER_STOP = 0;
    public static final double CONTROL_POWER_SHAKE = 0.15;

    public static final double Control_kS = 0.5;
    public static final double Control_kV = 0.12;
    public static final double Control_kA = 0.015;
    public static final double Control_kP = 4;
    public static final double Control_kI = 0;
    public static final double Control_kD = 0.05;
    public static final double Control_kG = 0;

    public static final double Control_CruiseVelocity = 80.0;
    public static final double Control_Acceleration = 160.0;

    public static final double Control_StatorCurrentLimit = 60;
    public static final double Control_SupplyCurrentLimit = 40;

    public static final double CONTROL_POS_IN = 0;
    public static final double CONTROL_POS_HELP_SHOOT = 3.5;
    public static final double CONTROL_POS_OUT = 5.5;

    public static final double CONTROL_POS_TOLERANCE = 0.2;
    public static final double HELP_TOGGLE_PERIOD = 0.4; // seconds

    // OTHER //
    public static final double SimulationMotorGearRatio = 12.8;

    // SysId //
    public static final double Control_SysIdQuasistatic = 0.5;
    public static final double Control_SysIdDynamic = 1.5;
}
