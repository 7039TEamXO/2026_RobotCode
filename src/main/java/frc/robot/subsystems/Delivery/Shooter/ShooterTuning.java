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

    private static final LoggedNetworkNumber Entry_Velocity_Roll = new LoggedNetworkNumber("/Tuning/Shooter/(Entry) Velocity Roll", ShooterConstants.ENTRY_VELOCITY_ROLL);
    private static final LoggedNetworkNumber Entry_Velocity_Reverse = new LoggedNetworkNumber("/Tuning/Shooter/(Entry) Velocity Reverse", ShooterConstants.ENTRY_VELOCITY_REVERSE);
    private static final LoggedNetworkNumber Entry_Velocity_Min = new LoggedNetworkNumber("/Tuning/Shooter/(Entry) Minimal Velocity", ShooterConstants.ENTRY_VELOCITY_MIN);

    private static final LoggedNetworkNumber Angle_kS = new LoggedNetworkNumber("/Tuning/Shooter/(Angle) kS", ShooterConstants.Angle_kS);
    private static final LoggedNetworkNumber Angle_kV = new LoggedNetworkNumber("/Tuning/Shooter/(Angle) kV", ShooterConstants.Angle_kV);
    private static final LoggedNetworkNumber Angle_kA = new LoggedNetworkNumber("/Tuning/Shooter/(Angle) kA", ShooterConstants.Angle_kA);
    private static final LoggedNetworkNumber Angle_kP = new LoggedNetworkNumber("/Tuning/Shooter/(Angle) kP", ShooterConstants.Angle_kP);
    private static final LoggedNetworkNumber Angle_kI = new LoggedNetworkNumber("/Tuning/Shooter/(Angle) kI", ShooterConstants.Angle_kI);
    private static final LoggedNetworkNumber Angle_kD = new LoggedNetworkNumber("/Tuning/Shooter/(Angle) kD", ShooterConstants.Angle_kD);

    private static final LoggedNetworkNumber Angle_CruiseVelocity = new LoggedNetworkNumber("/Tuning/Shooter/(Angle) Motion Magic Cruise Velocity", ShooterConstants.Angle_CruiseVelocity);
    private static final LoggedNetworkNumber Angle_Acceleration = new LoggedNetworkNumber("/Tuning/Shooter/(Angle) Motion Magic Acceleration", ShooterConstants.Angle_Acceleration);
    private static final LoggedNetworkNumber Angle_Jerk = new LoggedNetworkNumber("/Tuning/Shooter/(Angle) Motion Magic Jerk", ShooterConstants.Angle_Jerk);

    private static final LoggedNetworkNumber Angle_Intake_Help_Pos = new LoggedNetworkNumber("/Tuning/Shooter/(Angle) Intake Help Position", ShooterConstants.ANGLE_INTAKE_HELP_POS);
    private static final LoggedNetworkNumber Angle_Neutral_Zone_Pos = new LoggedNetworkNumber("/Tuning/Shooter/(Angle) Neutral Zone Position", ShooterConstants.ANGLE_NEUTRAL_ZONE_POS);

    private static final LoggedNetworkNumber Shoot_kS = new LoggedNetworkNumber("/Tuning/Shooter/(Shoot) kS", ShooterConstants.Shoot_kS);
    private static final LoggedNetworkNumber Shoot_kV = new LoggedNetworkNumber("/Tuning/Shooter/(Shoot) kV", ShooterConstants.Shoot_kV);
    private static final LoggedNetworkNumber Shoot_kA = new LoggedNetworkNumber("/Tuning/Shooter/(Shoot) kA", ShooterConstants.Shoot_kA);
    private static final LoggedNetworkNumber Shoot_kP = new LoggedNetworkNumber("/Tuning/Shooter/(Shoot) kP", ShooterConstants.Shoot_kP);
    private static final LoggedNetworkNumber Shoot_kI = new LoggedNetworkNumber("/Tuning/Shooter/(Shoot) kI", ShooterConstants.Shoot_kI);
    private static final LoggedNetworkNumber Shoot_kD = new LoggedNetworkNumber("/Tuning/Shooter/(Shoot) kD", ShooterConstants.Shoot_kD);

    private static final LoggedNetworkNumber Shoot_Velocity_Idle = new LoggedNetworkNumber("/Tuning/Shooter/(Shoot) Velocity Idle", ShooterConstants.SHOOT_VELOCITY_IDLE);
    private static final LoggedNetworkNumber Shoot_Velocity_Help = new LoggedNetworkNumber("/Tuning/Shooter/(Shoot) Velocity Help", ShooterConstants.SHOOT_VELOCITY_HELP);
    private static final LoggedNetworkNumber Shoot_Velocity_Neutral_Zone = new LoggedNetworkNumber("/Tuning/Shooter/(Shoot) Velocity Neutral Zone", ShooterConstants.SHOOT_VELOCITY_NEUTRAL_ZONE);
    private static final LoggedNetworkNumber Shoot_Velocity_Tolerance = new LoggedNetworkNumber("/Tuning/Shooter/(Shoot) Velocity Tolerance (rot/s)", ShooterConstants.SHOOT_VELOCITY_TOLERANCE);
    private static final LoggedNetworkNumber Shoot_Velocity_Min = new LoggedNetworkNumber("/Tuning/Shooter/(Shoot) Minimal Velocity", ShooterConstants.SHOOT_VELOCITY_MIN);

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

    public static final double ENTRY_VELOCITY_ROLL_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? Entry_Velocity_Roll.get() : ShooterConstants.ENTRY_VELOCITY_ROLL;
    }

    public static final double ENTRY_VELOCITY_REVERSE_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? Entry_Velocity_Reverse.get() : ShooterConstants.ENTRY_VELOCITY_REVERSE;
    }

    public static final double ENTRY_VELOCITY_MIN_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? Entry_Velocity_Min.get() : ShooterConstants.ENTRY_VELOCITY_MIN;
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

    public static final double Angle_CruiseVelocity_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? Angle_CruiseVelocity.get() : ShooterConstants.Angle_CruiseVelocity;
    }

    public static final double Angle_Acceleration_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? Angle_Acceleration.get() : ShooterConstants.Angle_Acceleration;
    }

    public static final double Angle_Jerk_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? Angle_Jerk.get() : ShooterConstants.Angle_Jerk;
    }

    public static final double ANGLE_INTAKE_HELP_POS_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? Angle_Intake_Help_Pos.get() : ShooterConstants.ANGLE_INTAKE_HELP_POS;
    }

    public static final double ANGLE_NEUTRAL_ZONE_POS_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? Angle_Neutral_Zone_Pos.get() : ShooterConstants.ANGLE_NEUTRAL_ZONE_POS;
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

    public static final double SHOOT_VELOCITY_IDLE_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? Shoot_Velocity_Idle.get() : ShooterConstants.SHOOT_VELOCITY_IDLE;
    }

    public static final double SHOOT_VELOCITY_HELP_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? Shoot_Velocity_Help.get() : ShooterConstants.SHOOT_VELOCITY_HELP;
    }

    public static final double SHOOT_VELOCITY_NEUTRAL_ZONE_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? Shoot_Velocity_Neutral_Zone.get() : ShooterConstants.SHOOT_VELOCITY_NEUTRAL_ZONE;
    }

    public static final double SHOOT_VELOCITY_TOLERANCE_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? Shoot_Velocity_Tolerance.get() : ShooterConstants.SHOOT_VELOCITY_TOLERANCE;
    }

    public static final double SHOOT_VELOCITY_MIN_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? Shoot_Velocity_Min.get() : ShooterConstants.SHOOT_VELOCITY_MIN;
    }
}