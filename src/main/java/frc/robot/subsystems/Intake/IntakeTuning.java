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
    private static final LoggedNetworkNumber Control_CruiseVelocity = new LoggedNetworkNumber("/Tuning/Intake/(Control) Motion Magic Cruise Velocity", IntakeConstants.Control_CruiseVelocity);
    private static final LoggedNetworkNumber Control_Acceleration = new LoggedNetworkNumber("/Tuning/Intake/(Control) Motion Magic Acceleration", IntakeConstants.Control_Acceleration);
    private static final LoggedNetworkNumber Control_Jerk = new LoggedNetworkNumber("/Tuning/Intake/(Control) Motion Magic Jerk", IntakeConstants.Control_Jerk);
    private static final LoggedNetworkNumber Control_kP = new LoggedNetworkNumber("/Tuning/Intake/(Control) Slot0 kP", IntakeConstants.Control_kP);
    private static final LoggedNetworkNumber Control_kV = new LoggedNetworkNumber("/Tuning/Intake/(Control) Slot0 kV", IntakeConstants.Control_kV);

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

    public static final double CONTROL_CruiseVelocity_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? Control_CruiseVelocity.get() : IntakeConstants.Control_CruiseVelocity;
    }

    public static final double CONTROL_Acceleration_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? Control_Acceleration.get() : IntakeConstants.Control_Acceleration;
    }

    public static final double CONTROL_Jerk_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? Control_Jerk.get() : IntakeConstants.Control_Jerk;
    }

    public static final double CONTROL_kP_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? Control_kP.get() : IntakeConstants.Control_kP;
    }

    public static final double CONTROL_kV_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? Control_kV.get() : IntakeConstants.Control_kV;
    }
}
