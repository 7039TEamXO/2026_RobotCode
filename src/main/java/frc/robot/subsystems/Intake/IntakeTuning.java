package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import frc.robot.Constants;
import frc.robot.Constants.TuningMode;

public class IntakeTuning {
    private static final LoggedNetworkNumber Intake_kS = new LoggedNetworkNumber("/Tuning/Intake/(Intake) kS", IntakeConstants.Intake_kS);
    private static final LoggedNetworkNumber Intake_kV = new LoggedNetworkNumber("/Tuning/Intake/(Intake) kV", IntakeConstants.Intake_kV);
    private static final LoggedNetworkNumber Intake_kA = new LoggedNetworkNumber("/Tuning/Intake/(Intake) kA", IntakeConstants.Intake_kA);
    private static final LoggedNetworkNumber Intake_kP = new LoggedNetworkNumber("/Tuning/Intake/(Intake) kP", IntakeConstants.Intake_kP);
    private static final LoggedNetworkNumber Intake_kI = new LoggedNetworkNumber("/Tuning/Intake/(Intake) kI", IntakeConstants.Intake_kI);
    private static final LoggedNetworkNumber Intake_kD = new LoggedNetworkNumber("/Tuning/Intake/(Intake) kD", IntakeConstants.Intake_kD);

    private static final LoggedNetworkNumber Control_kS = new LoggedNetworkNumber("/Tuning/Intake/(Control) kS", IntakeConstants.Control_kS);
    private static final LoggedNetworkNumber Control_kV = new LoggedNetworkNumber("/Tuning/Intake/(Control) kV", IntakeConstants.Control_kV);
    private static final LoggedNetworkNumber Control_kA = new LoggedNetworkNumber("/Tuning/Intake/(Control) kA", IntakeConstants.Control_kA);
    private static final LoggedNetworkNumber Control_kP = new LoggedNetworkNumber("/Tuning/Intake/(Control) kP", IntakeConstants.Control_kP);
    private static final LoggedNetworkNumber Control_kI = new LoggedNetworkNumber("/Tuning/Intake/(Control) kI", IntakeConstants.Control_kI);
    private static final LoggedNetworkNumber Control_kD = new LoggedNetworkNumber("/Tuning/Intake/(Control) kD", IntakeConstants.Control_kD);

    private static final LoggedNetworkNumber Control_Expo_CV = new LoggedNetworkNumber("/Tuning/Intake/(Control) Motion Magic CV", IntakeConstants.Control_Expo_CV);
    private static final LoggedNetworkNumber Control_Expo_kV = new LoggedNetworkNumber("/Tuning/Intake/(Control) Motion Magic kV", IntakeConstants.Control_Expo_kV);
    private static final LoggedNetworkNumber Control_Expo_kA = new LoggedNetworkNumber("/Tuning/Intake/(Control) Motion Magic kA", IntakeConstants.Control_Expo_kA);

    private static final LoggedNetworkNumber Power_Intake = new LoggedNetworkNumber("/Tuning/Shooter/Intake Power", IntakeConstants.INTAKE_POWER_INTAKE);
    private static final LoggedNetworkNumber Power_Deplete = new LoggedNetworkNumber("/Tuning/Shooter/Deplete Power", IntakeConstants.INTAKE_POWER_DEPLETE);

    public static final double Intake_kS_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? Intake_kS.get() : IntakeConstants.Intake_kS;
    }

    public static final double Intake_kV_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? Intake_kV.get() : IntakeConstants.Intake_kV;
    }

    public static final double Intake_kA_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? Intake_kA.get() : IntakeConstants.Intake_kA;
    }

    public static final double Intake_kP_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? Intake_kP.get() : IntakeConstants.Intake_kP;
    }

    public static final double Intake_kI_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? Intake_kI.get() : IntakeConstants.Intake_kI;
    }

    public static final double Intake_kD_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? Intake_kD.get() : IntakeConstants.Intake_kD;
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

    public static final double Control_Expo_CV_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? Control_Expo_CV.get() : IntakeConstants.Control_Expo_CV;
    }

    public static final double Control_Expo_kV_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? Control_Expo_kV.get() : IntakeConstants.Control_Expo_kV;
    }

    public static final double Control_Expo_kA_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? Control_Expo_kA.get() : IntakeConstants.Control_Expo_kA;
    }

    public static final double Power_Intake_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? Power_Intake.get() : IntakeConstants.INTAKE_POWER_INTAKE;
    }

    public static final double Power_Deplete_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? Power_Deplete.get() : IntakeConstants.INTAKE_POWER_DEPLETE;
    }
}
