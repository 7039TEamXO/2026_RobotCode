package frc.robot.subsystems.Delivery.Conveyor;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import frc.robot.Constants;
import frc.robot.Constants.TuningMode;

public class ConveyorTuning {
    private static final LoggedNetworkNumber Power_Guide = new LoggedNetworkNumber("/Tuning/Conveyor/Guide Power", ConveyorConstants.CONVEYOR_POWER_GUIDE);
    private static final LoggedNetworkNumber Power_Intake_Help = new LoggedNetworkNumber("/Tuning/Conveyor/Intake Help Power", ConveyorConstants.CONVEYOR_POWER_INTAKE_HELP);
    private static final LoggedNetworkNumber Power_Deplete = new LoggedNetworkNumber("/Tuning/Conveyor/Deplete Power", ConveyorConstants.CONVEYOR_POWER_DEPLETE);
    private static final LoggedNetworkNumber Unjam_Half_Cycle_Length = new LoggedNetworkNumber("/Tuning/Conveyor/Unjam Half-cycle Length", ConveyorConstants.UNJAM_HALF_CYCLE_LENGTH);

    public static final double Power_Guide_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? Power_Guide.get() : ConveyorConstants.CONVEYOR_POWER_GUIDE;
    }

    public static final double Power_Intake_Help_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? Power_Intake_Help.get() : ConveyorConstants.CONVEYOR_POWER_INTAKE_HELP;
    }

    public static final double Power_Deplete_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? Power_Deplete.get() : ConveyorConstants.CONVEYOR_POWER_DEPLETE;
    }

    public static final double Unjam_Half_Cycle_Length_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? Unjam_Half_Cycle_Length.get() : ConveyorConstants.UNJAM_HALF_CYCLE_LENGTH;
    }
}
