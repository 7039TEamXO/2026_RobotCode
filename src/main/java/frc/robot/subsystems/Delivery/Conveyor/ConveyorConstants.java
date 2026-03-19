package frc.robot.subsystems.Delivery.Conveyor;

public class ConveyorConstants {
    public static final int ConveyorMotorID = 13;

    public static final double StatorCurrentLimit = 70;
    public static final double SupplyCurrentLimit = 50;

    public static final double CONVEYOR_POWER_GUIDE = 0.8;
    public static final double CONVEYOR_POWER_INTAKE_HELP = -0.3;
    public static final double CONVEYOR_POWER_DEPLETE = -0.7;
    public static final double CONVEYOR_POWER_STOP = 0;


    public static final double UNJAM_HALF_CYCLE_LENGTH = 200; // in ms

    // OTHER //

    public static final double SimulationMotorGearRatio = 12.8;
}