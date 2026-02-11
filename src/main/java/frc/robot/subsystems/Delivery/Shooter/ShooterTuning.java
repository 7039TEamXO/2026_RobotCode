package frc.robot.subsystems.Delivery.Shooter;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import frc.robot.Constants;
import frc.robot.Constants.TuningMode;

public class ShooterTuning {
    private static final LoggedNetworkNumber Entry_kS = new LoggedNetworkNumber("/Tuning/Shooter/(Entry) kS", ShooterConstants.Entry_kS);
    private static final LoggedNetworkNumber Entry_kV = new LoggedNetworkNumber("/Tuning/Shooter/(Entry) kV", ShooterConstants.Entry_kV);
    private static final LoggedNetworkNumber Entry_kA = new LoggedNetworkNumber("/Tuning/Shooter/(Entry) kA", ShooterConstants.Entry_kA);
    private static final LoggedNetworkNumber Entry_kP = new LoggedNetworkNumber("/Tuning/Shooter/(Entry) kP", ShooterConstants.Entry_kP);
    private static final LoggedNetworkNumber Entry_kI = new LoggedNetworkNumber("/Tuning/Shooter/(Entry) kI", ShooterConstants.Entry_kI);
    private static final LoggedNetworkNumber Entry_kD = new LoggedNetworkNumber("/Tuning/Shooter/(Entry) kD", ShooterConstants.Entry_kD);

    private static final LoggedNetworkNumber Angle_kS = new LoggedNetworkNumber("/Tuning/Shooter/(Angle) kS", ShooterConstants.Angle_kS);
    private static final LoggedNetworkNumber Angle_kV = new LoggedNetworkNumber("/Tuning/Shooter/(Angle) kV", ShooterConstants.Angle_kV);
    private static final LoggedNetworkNumber Angle_kA = new LoggedNetworkNumber("/Tuning/Shooter/(Angle) kA", ShooterConstants.Angle_kA);
    private static final LoggedNetworkNumber Angle_kP = new LoggedNetworkNumber("/Tuning/Shooter/(Angle) kP", ShooterConstants.Angle_kP);
    private static final LoggedNetworkNumber Angle_kI = new LoggedNetworkNumber("/Tuning/Shooter/(Angle) kI", ShooterConstants.Angle_kI);
    private static final LoggedNetworkNumber Angle_kD = new LoggedNetworkNumber("/Tuning/Shooter/(Angle) kD", ShooterConstants.Angle_kD);

    private static final LoggedNetworkNumber Angle_Expo_CV = new LoggedNetworkNumber("/Tuning/Shooter/(Angle) Motion Magic CV", ShooterConstants.Angle_Expo_CV);
    private static final LoggedNetworkNumber Angle_Expo_kV = new LoggedNetworkNumber("/Tuning/Shooter/(Angle) Motion Magic kV", ShooterConstants.Angle_Expo_kV);
    private static final LoggedNetworkNumber Angle_Expo_kA = new LoggedNetworkNumber("/Tuning/Shooter/(Angle) Motion Magic kA", ShooterConstants.Angle_Expo_kA);

    private static final LoggedNetworkNumber Shoot_kS = new LoggedNetworkNumber("/Tuning/Shooter/(Shoot) kS", ShooterConstants.Shoot_kS);
    private static final LoggedNetworkNumber Shoot_kV = new LoggedNetworkNumber("/Tuning/Shooter/(Shoot) kV", ShooterConstants.Shoot_kV);
    private static final LoggedNetworkNumber Shoot_kA = new LoggedNetworkNumber("/Tuning/Shooter/(Shoot) kA", ShooterConstants.Shoot_kA);
    private static final LoggedNetworkNumber Shoot_kP = new LoggedNetworkNumber("/Tuning/Shooter/(Shoot) kP", ShooterConstants.Shoot_kP);
    private static final LoggedNetworkNumber Shoot_kI = new LoggedNetworkNumber("/Tuning/Shooter/(Shoot) kI", ShooterConstants.Shoot_kI);
    private static final LoggedNetworkNumber Shoot_kD = new LoggedNetworkNumber("/Tuning/Shooter/(Shoot) kD", ShooterConstants.Shoot_kD);

    private static final LoggedNetworkNumber Entry_Power_Roll = new LoggedNetworkNumber("/Tuning/Shooter/(Entry) Power Roll", ShooterConstants.ENTRY_POWER_ROLL);

    private static final LoggedNetworkNumber Shoot_Voltage_Idle = new LoggedNetworkNumber("/Tuning/Shooter/(Shoot) Voltage Idle", ShooterConstants.SHOOT_VOLTAGE_IDLE);
    private static final LoggedNetworkNumber Shoot_Voltage_Fire = new LoggedNetworkNumber("/Tuning/Shooter/(Shoot) Voltage Fire", ShooterConstants.SHOOT_VOLTAGE_FIRE); 
    private static final LoggedNetworkNumber Shoot_Voltage_Tolerance = new LoggedNetworkNumber("/Tuning/Shooter/(Shoot) Voltage Tolerance", ShooterConstants.SHOOT_VOLTAGE_TOLERANCE);

    public static final double Entry_kS_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? Entry_kS.get() : ShooterConstants.Entry_kS;
    }

    public static final double Entry_kV_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? Entry_kV.get() : ShooterConstants.Entry_kV;
    }

    public static final double Entry_kA_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? Entry_kA.get() : ShooterConstants.Entry_kA;
    }

    public static final double Entry_kP_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? Entry_kP.get() : ShooterConstants.Entry_kP;
    }

    public static final double Entry_kI_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? Entry_kI.get() : ShooterConstants.Entry_kI;
    }

    public static final double Entry_kD_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? Entry_kD.get() : ShooterConstants.Entry_kD;
    }

    public static final double Angle_kS_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? Angle_kS.get() : ShooterConstants.Angle_kS;
    }

    public static final double Angle_kV_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? Angle_kV.get() : ShooterConstants.Angle_kV;
    }

    public static final double Angle_kA_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? Angle_kA.get() : ShooterConstants.Angle_kA;
    }

    public static final double Angle_kP_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? Angle_kP.get() : ShooterConstants.Angle_kP;
    }

    public static final double Angle_kI_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? Angle_kI.get() : ShooterConstants.Angle_kI;
    }

    public static final double Angle_kD_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? Angle_kD.get() : ShooterConstants.Angle_kD;
    }

    public static final double Angle_Expo_CV_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? Angle_Expo_CV.get() : ShooterConstants.Angle_Expo_CV;
    }

    public static final double Angle_Expo_kV_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? Angle_Expo_kV.get() : ShooterConstants.Angle_Expo_kV;
    }

    public static final double Angle_Expo_kA_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? Angle_Expo_kA.get() : ShooterConstants.Angle_Expo_kA;
    }

    public static final double Shoot_kS_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? Shoot_kS.get() : ShooterConstants.Shoot_kS;
    }

    public static final double Shoot_kV_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? Shoot_kV.get() : ShooterConstants.Shoot_kV;
    }

    public static final double Shoot_kA_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? Shoot_kA.get() : ShooterConstants.Shoot_kA;
    }

    public static final double Shoot_kP_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? Shoot_kP.get() : ShooterConstants.Shoot_kP;
    }

    public static final double Shoot_kI_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? Shoot_kI.get() : ShooterConstants.Shoot_kI;
    }

    public static final double Shoot_kD_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? Shoot_kD.get() : ShooterConstants.Shoot_kD;
    }

    public static final double Entry_Power_Roll_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? Entry_Power_Roll.get() : ShooterConstants.ENTRY_POWER_ROLL;
    }

    public static final double Shoot_Voltage_Fire_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? Shoot_Voltage_Fire.get() : ShooterConstants.SHOOT_VOLTAGE_FIRE;
    }

    public static final double Shoot_Voltage_Idle_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? Shoot_Voltage_Idle.get() : ShooterConstants.SHOOT_VOLTAGE_IDLE;
    }

    public static final double Shoot_Voltage_Tolerance_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? Shoot_Voltage_Tolerance.get() : ShooterConstants.SHOOT_VOLTAGE_TOLERANCE;
    }
}