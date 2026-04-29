package frc.robot.subsystems.Intake;

public class IntakeConstants {
    public static final int IntakeMotorID = 14;
    public static final int ControlMotorID = 12;
    
    // INTAKE MOTOR //
    public static final double Intake_StatorCurrentLimit = 85;
    public static final double Intake_SupplyCurrentLimit = 60;

    public static final double INTAKE_CURRENT_FREE = 40;
    public static final double INTAKE_CURRENT_KICK = 80;

    public static final double INTAKE_POWER_STOP = 0;
    public static final double INTAKE_POWER_INTAKE = -1;
    public static final double INTAKE_POWER_HELP_SHOOT = 55;
    public static final double INTAKE_POWER_DEPLETE = -55;

    // CONTROL MOTOR //
    public static final double CONTROL_POWER_STOP = 0;
    public static final double CONTROL_POWER_SHAKE = 0.37;

    public static final double Control_kS = 0.6;
    public static final double Control_kV = 0.16;
    public static final double Control_kA = 0.015;
    public static final double Control_kP = 1.9;
    public static final double Control_kI = 0.02;
    public static final double Control_kD = 0;
    public static final double Control_kG = 0;

    public static final double Control_CruiseVelocity = 100.0;
    public static final double Control_Acceleration = 65.0;

    public static final double Control_StatorCurrentLimit = 50;
    public static final double Control_SupplyCurrentLimit = 50;

    public static final double CONTROL_POS_IN = 0;
    public static final double CONTROL_POS_HELP_SHOOT = 3.2;
    public static final double CONTROL_POS_OUT = 5.45;

    public static final double CONTROL_POS_TOLERANCE = 0.2;

    public static final double HELP_TOGGLE_PERIOD = 0.4; // seconds
    public static final double INITIAL_STOP_TIME = 1.5;  // seconds

    // OTHER //
    public static final double SimulationMotorGearRatio = 12.8;

    // SysId //
    public static final double Control_SysIdQuasistatic = 0.5;
    public static final double Control_SysIdDynamic = 1.5;
}
