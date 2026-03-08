package frc.robot.subsystems.Intake;

public class IntakeConstants {
    public static final int IntakeMotorID = 14;
    public static final int ControlMotorID = 12;
    
    // INTAKE MOTOR //
    public static final double Intake_StatorCurrentLimit = 60;
    public static final double Intake_SupplyCurrentLimit = 60;

    public static final double INTAKE_POWER_STOP = 0;
    public static final double INTAKE_POWER_INTAKE = -0.8;
    public static final double INTAKE_POWER_HELP_SHOOT = -0.6;
    public static final double INTAKE_POWER_DEPLETE = 0.8;

    // CONTROL MOTOR //
    public static final double CONTROL_POWER_STOP = 0;
    public static final double CONTROL_POWER_SHAKE = 0.15;
    public static final double CONTROL_POWER_OPEN = 0.2;
    public static final double CONTROL_POWER_CLOSE = -0.25;

    public static final double Control_StatorCurrentLimit = 40;
    public static final double Control_SupplyCurrentLimit = 40;

    public static final double CONTROL_POS_IN = 0; // was -0.1
    public static final double CONTROL_POS_HELP_SHOOT = 3.5;
    public static final double CONTROL_POS_OUT = 5; //was 5.4

    public static final double CONTROL_POS_TOLERANCE = 0.2;
    public static final double HELP_TOGGLE_PERIOD = 0.4; // seconds

    // OTHER //
    public static final double SimulationMotorGearRatio = 12.8;

    // SysId //
    public static final double Control_SysIdQuasistatic = 0.5;
    public static final double Control_SysIdDynamic = 1.5;

    // Motion Magic defaults for control (units follow Talon/encoder native units used
    // elsewhere in the project - tuned values can be changed via IntakeTuning)
    public static final double Control_kP = 0.7;
    public static final double Control_kV = 0.1;

    public static final double Control_CruiseVelocity = 40.0;
    public static final double Control_Acceleration = 40.0;
    public static final double Control_Jerk = 400.0;
}
