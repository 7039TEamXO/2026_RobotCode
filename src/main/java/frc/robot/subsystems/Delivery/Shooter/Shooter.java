package frc.robot.subsystems.Delivery.Shooter;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;
// import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.TuningMode;
// import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.IO.ShooterIO;
import frc.robot.subsystems.IO.ShooterIO.ShooterIOInputs;

public class Shooter implements Subsystem {
    private final ShooterIO io;
    private final ShooterIOInputs inputs;

    public Shooter(ShooterIO _io) {
        io = _io;
        inputs = new ShooterIOInputs();

        setMotorConfigs();

        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
    }

    public void operate(ShooterState state) {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);

        if(Constants.GetTuningMode() == TuningMode.ACTIVE) setMotorConfigs();

        double Entry_Power = ShooterConstants.ENTRY_POWER_STOP;
        double Angle_Position = calculateShooterPosition();
        double Shoot_Power = ShooterTuning.Shoot_Idle_get();
        
        switch(state) {
            case SPEED_UP:
                Shoot_Power = ShooterTuning.Shoot_Fire_get();
                break;
            case SHOOT:
                Entry_Power = ShooterTuning.Entry_Power_get();
                Shoot_Power = ShooterTuning.Shoot_Fire_get();
                break;
            default:
            case IDLE:
                // Do nothing
                break;
        }

        io.setEntryMotionMagic(new DutyCycleOut(Entry_Power));
        io.setAngleMotionMagic(new MotionMagicExpoVoltage(Angle_Position));
        io.setMasterMotionMagic(new DutyCycleOut(Shoot_Power));
    }

    private double calculateShooterPosition() {
        // Pose2d pose = SubsystemManager.getDrivebase().getPose();

        return 0; // TO-DO
    }

    private void setEntryMotorConfigs() {
        TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
        Slot0Configs slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = ShooterTuning.Entry_kS_get();
        slot0Configs.kV = ShooterTuning.Entry_kV_get();
        slot0Configs.kA = ShooterTuning.Entry_kA_get();
        slot0Configs.kP = ShooterTuning.Entry_kP_get();
        slot0Configs.kI = ShooterTuning.Entry_kI_get();
        slot0Configs.kD = ShooterTuning.Entry_kD_get();

        MotionMagicConfigs motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = ShooterTuning.Entry_Expo_CV_get();
        motionMagicConfigs.MotionMagicExpo_kV = ShooterTuning.Entry_Expo_kV_get();
        motionMagicConfigs.MotionMagicExpo_kA = ShooterTuning.Entry_Expo_kA_get();

        talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        talonFXConfigs.CurrentLimits.StatorCurrentLimit = ShooterConstants.Entry_StatorCurrentLimit;
        talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        talonFXConfigs.CurrentLimits.SupplyCurrentLimit = ShooterConstants.Entry_SupplyCurrentLimit;
        talonFXConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

        io.applyEntryTalonFXConfig(talonFXConfigs);
    }

    private void setAngleMotorConfigs() {
        TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
        Slot0Configs slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = ShooterTuning.Angle_kS_get();
        slot0Configs.kV = ShooterTuning.Angle_kV_get();
        slot0Configs.kA = ShooterTuning.Angle_kA_get();
        slot0Configs.kP = ShooterTuning.Angle_kP_get();
        slot0Configs.kI = ShooterTuning.Angle_kI_get();
        slot0Configs.kD = ShooterTuning.Angle_kD_get();

        MotionMagicConfigs motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = ShooterTuning.Angle_Expo_CV_get();
        motionMagicConfigs.MotionMagicExpo_kV = ShooterTuning.Angle_Expo_kV_get();
        motionMagicConfigs.MotionMagicExpo_kA = ShooterTuning.Angle_Expo_kA_get();

        talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        talonFXConfigs.CurrentLimits.StatorCurrentLimit = ShooterConstants.Angle_StatorCurrentLimit;
        talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        talonFXConfigs.CurrentLimits.SupplyCurrentLimit = ShooterConstants.Angle_SupplyCurrentLimit;
        talonFXConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

        io.applyAngleTalonFXConfig(talonFXConfigs);
    }

    private void setShootMotorsConfigs() {
        io.setSlaveMotionMagic(new Follower(ShooterConstants.ShooterMasterMotorID, MotorAlignmentValue.Aligned));

        TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
        Slot0Configs slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = ShooterTuning.Shoot_kS_get();
        slot0Configs.kV = ShooterTuning.Shoot_kV_get();
        slot0Configs.kA = ShooterTuning.Shoot_kA_get();
        slot0Configs.kP = ShooterTuning.Shoot_kP_get();
        slot0Configs.kI = ShooterTuning.Shoot_kI_get();
        slot0Configs.kD = ShooterTuning.Shoot_kD_get();

        MotionMagicConfigs motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = ShooterTuning.Shoot_Expo_CV_get();
        motionMagicConfigs.MotionMagicExpo_kV = ShooterTuning.Shoot_Expo_kV_get();
        motionMagicConfigs.MotionMagicExpo_kA = ShooterTuning.Shoot_Expo_kA_get();

        talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        talonFXConfigs.CurrentLimits.StatorCurrentLimit = ShooterConstants.Shoot_StatorCurrentLimit;
        talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        talonFXConfigs.CurrentLimits.SupplyCurrentLimit = ShooterConstants.Shoot_SupplyCurrentLimit;
        talonFXConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

        io.applyMasterTalonFXConfig(talonFXConfigs);

        TalonFXConfiguration slaveTalonFXConfigs = new TalonFXConfiguration();

        slaveTalonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        slaveTalonFXConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        slaveTalonFXConfigs.CurrentLimits.StatorCurrentLimit = ShooterConstants.Shoot_StatorCurrentLimit;
        slaveTalonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        slaveTalonFXConfigs.CurrentLimits.SupplyCurrentLimit = ShooterConstants.Shoot_SupplyCurrentLimit;
        slaveTalonFXConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

        io.applySlaveTalonFXConfig(slaveTalonFXConfigs);
    }

    private void setMotorConfigs() {
        setEntryMotorConfigs();
        setAngleMotorConfigs();
        setShootMotorsConfigs();
    }
}
