package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import frc.robot.Constants;
import frc.robot.Constants.TuningMode;

public class IntakeTuning {
    private static final LoggedNetworkNumber Power_Intake = new LoggedNetworkNumber("/Tuning/Intake/Intake Power", IntakeConstants.INTAKE_POWER_INTAKE);
    private static final LoggedNetworkNumber Power_Help_Shoot = new LoggedNetworkNumber("/Tuning/Intake/Help Shoot Power", IntakeConstants.INTAKE_POWER_HELP_SHOOT);
    private static final LoggedNetworkNumber Power_Deplete = new LoggedNetworkNumber("/Tuning/Intake/Deplete Power", IntakeConstants.INTAKE_POWER_DEPLETE);
    
    private static final LoggedNetworkNumber Control_Power_Shake = new LoggedNetworkNumber("/Tuning/Intake/Control Power Shake", IntakeConstants.CONTROL_POWER_SHAKE);
    private static final LoggedNetworkNumber Control_CruiseVelocity = new LoggedNetworkNumber("/Tuning/Intake/(Control) Motion Magic Cruise Velocity", IntakeConstants.Control_CruiseVelocity);
    private static final LoggedNetworkNumber Control_Acceleration = new LoggedNetworkNumber("/Tuning/Intake/(Control) Motion Magic Acceleration", IntakeConstants.Control_Acceleration);

    private static final LoggedNetworkNumber Control_kS = new LoggedNetworkNumber("/Tuning/Intake/(Control) kS", IntakeConstants.Control_kS);
    private static final LoggedNetworkNumber Control_kV = new LoggedNetworkNumber("/Tuning/Intake/(Control) kV", IntakeConstants.Control_kV);
    private static final LoggedNetworkNumber Control_kA = new LoggedNetworkNumber("/Tuning/Intake/(Control) kA", IntakeConstants.Control_kA);
    private static final LoggedNetworkNumber Control_kP = new LoggedNetworkNumber("/Tuning/Intake/(Control) kP", IntakeConstants.Control_kP);
    private static final LoggedNetworkNumber Control_kI = new LoggedNetworkNumber("/Tuning/Intake/(Control) kI", IntakeConstants.Control_kI);
    private static final LoggedNetworkNumber Control_kD = new LoggedNetworkNumber("/Tuning/Intake/(Control) kD", IntakeConstants.Control_kD);
    private static final LoggedNetworkNumber Control_kG = new LoggedNetworkNumber("/Tuning/Intake/(Control) kG", IntakeConstants.Control_kG);

    public static final double POWER_INTAKE_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? Power_Intake.get() : IntakeConstants.INTAKE_POWER_INTAKE;
    }

    public static final double POWER_HELP_SHOOT_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? Power_Help_Shoot.get() : IntakeConstants.INTAKE_POWER_HELP_SHOOT;
    }

    public static final double POWER_DEPLETE_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? Power_Deplete.get() : IntakeConstants.INTAKE_POWER_DEPLETE;
    }

    public static final double CONTROL_POWER_SHAKE_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? Control_Power_Shake.get() : IntakeConstants.CONTROL_POWER_SHAKE;
    }

    public static final double Control_CruiseVelocity_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? Control_CruiseVelocity.get() : IntakeConstants.Control_CruiseVelocity;
    }

    public static final double Control_Acceleration_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? Control_Acceleration.get() : IntakeConstants.Control_Acceleration;
    }

    public static final double Control_kS_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? Control_kS.get() : IntakeConstants.Control_kS;
    }

    public static final double Control_kV_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? Control_kV.get() : IntakeConstants.Control_kV;
    }

    public static final double Control_kA_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? Control_kA.get() : IntakeConstants.Control_kA;
    }

    public static final double Control_kP_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? Control_kP.get() : IntakeConstants.Control_kP;
    }

    public static final double Control_kI_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? Control_kI.get() : IntakeConstants.Control_kI;
    }

    public static final double Control_kD_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? Control_kD.get() : IntakeConstants.Control_kD;
    }

    public static final double Control_kG_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? Control_kG.get() : IntakeConstants.Control_kG;
    }
}
