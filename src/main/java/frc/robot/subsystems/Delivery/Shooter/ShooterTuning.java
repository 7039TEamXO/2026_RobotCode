package frc.robot.subsystems.Delivery.Shooter;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import frc.robot.Constants;
import frc.robot.Constants.TuningMode;

public class ShooterTuning {
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

    private static final LoggedNetworkNumber Shoot_kS = new LoggedNetworkNumber("/Tuning/Shooter/(Shoot) kS", ShooterConstants.Shoot_kS);
    private static final LoggedNetworkNumber Shoot_kV = new LoggedNetworkNumber("/Tuning/Shooter/(Shoot) kV", ShooterConstants.Shoot_kV);
    private static final LoggedNetworkNumber Shoot_kA = new LoggedNetworkNumber("/Tuning/Shooter/(Shoot) kA", ShooterConstants.Shoot_kA);
    private static final LoggedNetworkNumber Shoot_kP = new LoggedNetworkNumber("/Tuning/Shooter/(Shoot) kP", ShooterConstants.Shoot_kP);
    private static final LoggedNetworkNumber Shoot_kI = new LoggedNetworkNumber("/Tuning/Shooter/(Shoot) kI", ShooterConstants.Shoot_kI);
    private static final LoggedNetworkNumber Shoot_kD = new LoggedNetworkNumber("/Tuning/Shooter/(Shoot) kD", ShooterConstants.Shoot_kD);

    private static final LoggedNetworkNumber Entry_Power_Roll = new LoggedNetworkNumber("/Tuning/Shooter/(Entry) Power Roll", ShooterConstants.ENTRY_POWER_ROLL);
    private static final LoggedNetworkNumber Entry_Power_Reverse = new LoggedNetworkNumber("/Tuning/Shooter/(Entry) Power Reverse", ShooterConstants.ENTRY_POWER_REVERSE);

    private static final LoggedNetworkNumber Shoot_Velocity_Idle = new LoggedNetworkNumber("/Tuning/Shooter/(Shoot) Velocity Idle rot_s", ShooterConstants.SHOOT_VELOCITY_IDLE);
    private static final LoggedNetworkNumber Shoot_Velocity_Fire = new LoggedNetworkNumber("/Tuning/Shooter/(Shoot) Velocity Fire rot_s", ShooterConstants.SHOOT_VELOCITY_FIRE);
    private static final LoggedNetworkNumber Shoot_Velocity_Tolerance = new LoggedNetworkNumber("/Tuning/Shooter/(Shoot) Velocity Tolerance rot_s", ShooterConstants.SHOOT_VELOCITY_TOLERANCE);
    private static final LoggedNetworkNumber Shoot_Velocity_Help = new LoggedNetworkNumber("/Tuning/Shooter/(Shoot) Velocity Help rot_s", ShooterConstants.SHOOT_VELOCITY_HELP);

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

    public static final double Angle_Intake_Help_Pos_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? Angle_Intake_Help_Pos.get() : ShooterConstants.ANGLE_INTAKE_HELP_POS;
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

    public static final double Entry_Power_Reverse_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? Entry_Power_Reverse.get() : ShooterConstants.ENTRY_POWER_REVERSE;
    }

    public static final double Shoot_Velocity_Idle_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? Shoot_Velocity_Idle.get() : ShooterConstants.SHOOT_VELOCITY_IDLE;
    }

    public static final double Shoot_Velocity_Fire_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? Shoot_Velocity_Fire.get() : ShooterConstants.SHOOT_VELOCITY_FIRE;
    }

    public static final double Shoot_Velocity_Tolerance_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? Shoot_Velocity_Tolerance.get() : ShooterConstants.SHOOT_VELOCITY_TOLERANCE;
    }

    public static final double Shoot_Velocity_Help_get() {
        return Constants.GetTuningMode() != TuningMode.IGNORE ? Shoot_Velocity_Help.get() : ShooterConstants.SHOOT_VELOCITY_HELP;
    }
}