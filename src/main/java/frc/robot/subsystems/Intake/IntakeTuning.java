package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import frc.robot.Constants;
import frc.robot.Constants.TuningMode;

public class IntakeTuning {
    private static final LoggedNetworkNumber Power_Intake = new LoggedNetworkNumber("/Tuning/Intake/Intake Power", IntakeConstants.INTAKE_POWER_INTAKE);
    private static final LoggedNetworkNumber Power_Help_Shoot = new LoggedNetworkNumber("/Tuning/Intake/Help Shoot Power", IntakeConstants.INTAKE_POWER_HELP_SHOOT);
    private static final LoggedNetworkNumber Power_Deplete = new LoggedNetworkNumber("/Tuning/Intake/Deplete Power", IntakeConstants.INTAKE_POWER_DEPLETE);
    
    private static final LoggedNetworkNumber Control_Power_Open = new LoggedNetworkNumber("/Tuning/Intake/Control Power Open", IntakeConstants.CONTROL_POWER_OPEN);
    private static final LoggedNetworkNumber Control_Power_Close = new LoggedNetworkNumber("/Tuning/Intake/Control Power Close", IntakeConstants.CONTROL_POWER_CLOSE);
    private static final LoggedNetworkNumber Control_Power_Shake = new LoggedNetworkNumber("/Tuning/Intake/Control Power Shake", IntakeConstants.CONTROL_POWER_SHAKE);

    public static final double POWER_INTAKE_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? Power_Intake.get() : IntakeConstants.INTAKE_POWER_INTAKE;
    }

    public static final double POWER_HELP_SHOOT_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? Power_Help_Shoot.get() : IntakeConstants.INTAKE_POWER_HELP_SHOOT;
    }

    public static final double POWER_DEPLETE_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? Power_Deplete.get() : IntakeConstants.INTAKE_POWER_DEPLETE;
    }

    public static final double CONTROL_POWER_OPEN_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? Control_Power_Open.get() : IntakeConstants.CONTROL_POWER_OPEN;
    }

    public static final double CONTROL_POWER_CLOSE_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? Control_Power_Close.get() : IntakeConstants.CONTROL_POWER_CLOSE;
    }

    public static final double CONTROL_POWER_SHAKE_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? Control_Power_Shake.get() : IntakeConstants.CONTROL_POWER_SHAKE;
    }
}
