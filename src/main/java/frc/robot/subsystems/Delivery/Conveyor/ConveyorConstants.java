package frc.robot.subsystems.Delivery.Conveyor;

public class ConveyorConstants {
    public static final int ConveyorMotorID = 18;

    public static final double StatorCurrentLimit = 50;
    public static final double SupplyCurrentLimit = 40;

    public static final double CONVEYOR_POWER_GUIDE = 1;
    public static final double CONVEYOR_POWER_INTAKE_HELP = 0.4;
    public static final double CONVEYOR_POWER_DEPLETE = -0.7;
    public static final double CONVEYOR_POWER_STOP = 0;

    public static final double UNJAM_HALF_CYCLE_LENGTH = 300; // in ms

    // OTHER //

    public static final double SimulationMotorGearRatio = 12.8;
}
