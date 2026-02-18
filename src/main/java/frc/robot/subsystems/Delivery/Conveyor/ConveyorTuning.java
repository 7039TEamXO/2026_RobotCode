package frc.robot.subsystems.Delivery.Conveyor;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import frc.robot.Constants;
import frc.robot.Constants.TuningMode;

public class ConveyorTuning {
    private static final LoggedNetworkNumber kS = new LoggedNetworkNumber("/Tuning/Conveyor/kS", ConveyorConstants.kS);
    private static final LoggedNetworkNumber kV = new LoggedNetworkNumber("/Tuning/Conveyor/kV", ConveyorConstants.kV);
    private static final LoggedNetworkNumber kA = new LoggedNetworkNumber("/Tuning/Conveyor/kA", ConveyorConstants.kA);
    private static final LoggedNetworkNumber kP = new LoggedNetworkNumber("/Tuning/Conveyor/kP", ConveyorConstants.kP);
    private static final LoggedNetworkNumber kI = new LoggedNetworkNumber("/Tuning/Conveyor/kI", ConveyorConstants.kI);
    private static final LoggedNetworkNumber kD = new LoggedNetworkNumber("/Tuning/Conveyor/kD", ConveyorConstants.kD);

    private static final LoggedNetworkNumber Power_Guide = new LoggedNetworkNumber("/Tuning/Conveyor/Guide Power", ConveyorConstants.CONVEYOR_POWER_GUIDE);
    private static final LoggedNetworkNumber Power_Intake_Help = new LoggedNetworkNumber("/Tuning/Conveyor/Intake Help Power", ConveyorConstants.CONVEYOR_POWER_INTAKE_HELP);
    private static final LoggedNetworkNumber Power_Deplete = new LoggedNetworkNumber("/Tuning/Conveyor/Deplete Power", ConveyorConstants.CONVEYOR_POWER_GUIDE);
    private static final LoggedNetworkNumber Unjam_Half_Cycle_Length = new LoggedNetworkNumber("/Tuning/Conveyor/Unjam Half-cycle Length", ConveyorConstants.UNJAM_HALF_CYCLE_LENGTH);

    public static final double kS_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? kS.get() : ConveyorConstants.kS;
    }

    public static final double kV_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? kV.get() : ConveyorConstants.kV;
    }

    public static final double kA_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? kA.get() : ConveyorConstants.kA;
    }

    public static final double kP_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? kP.get() : ConveyorConstants.kP;
    }

    public static final double kI_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? kI.get() : ConveyorConstants.kI;
    }

    public static final double kD_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? kD.get() : ConveyorConstants.kD;
    }

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
