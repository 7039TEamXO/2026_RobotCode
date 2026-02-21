package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import frc.robot.Constants;
import frc.robot.Constants.TuningMode;

public class IntakeTuning {
    private static final LoggedNetworkNumber Power_Intake = new LoggedNetworkNumber("/Tuning/Shooter/Intake Power", IntakeConstants.INTAKE_POWER_INTAKE);
    private static final LoggedNetworkNumber Power_Deplete = new LoggedNetworkNumber("/Tuning/Shooter/Deplete Power", IntakeConstants.INTAKE_POWER_DEPLETE);
    
    public static final double Power_Intake_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? Power_Intake.get() : IntakeConstants.INTAKE_POWER_INTAKE;
    }

    public static final double Power_Deplete_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? Power_Deplete.get() : IntakeConstants.INTAKE_POWER_DEPLETE;
    }
}
