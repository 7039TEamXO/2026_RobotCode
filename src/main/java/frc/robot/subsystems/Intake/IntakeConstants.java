package frc.robot.subsystems.Intake;

public class IntakeConstants {
    public static final int IntakeMotorID = 19;
    public static final int ControlMotorID = 16;
    
    // INTAKE MOTOR //
    public static final double Intake_StatorCurrentLimit = 30;
    public static final double Intake_SupplyCurrentLimit = 30;

    public static final double INTAKE_POWER_STOP = 0;
    public static final double INTAKE_POWER_INTAKE = -0.8;
    public static final double INTAKE_POWER_DEPLETE = 0.6;

    // CONTROL MOTOR //
    public static final double CONTROL_POWER_STOP = 0;
    public static final double CONTROL_POWER_OPEN = 0.15;
    public static final double CONTROL_POWER_CLOSE = -0.2;

    public static final double Control_StatorCurrentLimit = 70;
    public static final double Control_SupplyCurrentLimit = 50;

    public static final double CONTROL_POS_IN = -0.1;
    public static final double CONTROL_POS_HELP_SHOOT = 4;
    public static final double CONTROL_POS_OUT = 5.4;

    public static final double CONTROL_POS_TOLERANCE = 0.3;
    public static final double HELP_TOGGLE_PERIOD = 0.5; // seconds

    // OTHER //
    public static final double SimulationMotorGearRatio = 12.8;

    // SysId //
    public static final double Control_SysIdQuasistatic = 0.5;
    public static final double Control_SysIdDynamic = 1.5;
}
