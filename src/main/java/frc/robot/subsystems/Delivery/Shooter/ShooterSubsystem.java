package frc.robot.subsystems.Delivery.Shooter;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
// import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;
import frc.robot.Dashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.TuningMode;
import frc.robot.subsystems.IO.ShooterIO;
import frc.robot.subsystems.IO.ShooterIO.ShooterIOInputs;

public class ShooterSubsystem implements Subsystem {
    private final ShooterIO io;
    private final ShooterIOInputs inputs;

    public ShooterSubsystem(ShooterIO _io) {
        io = _io;
        inputs = new ShooterIOInputs();

        setMotorConfigs();
        io.setAngleMotorPosition(0);

        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);

        Entry_SysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.per(Second).of(ShooterConstants.Entry_SysIdQuasistatic), 
                Volts.of(ShooterConstants.Entry_SysIdDynamic), null, state -> {
                    SignalLogger.writeString("state", state.toString());
            }),
            new SysIdRoutine.Mechanism(v -> {
                io.setEntryRequest(new VoltageOut(v));

                io.updateInputs(inputs);
                Logger.processInputs("Shooter", inputs);
            }, null, this));

        Angle_SysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.per(Second).of(ShooterConstants.Angle_SysIdQuasistatic), 
                Volts.of(ShooterConstants.Angle_SysIdDynamic), null, state -> {
                    SignalLogger.writeString("state", state.toString());
            }),
            new SysIdRoutine.Mechanism(v -> {
                io.setAngleRequest(new VoltageOut(v));

                io.updateInputs(inputs);
                Logger.processInputs("Shooter", inputs);
            }, null, this));

        Shoot_SysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.per(Second).of(ShooterConstants.Shoot_SysIdQuasistatic), 
                Volts.of(ShooterConstants.Shoot_SysIdDynamic), null, state -> {
                    SignalLogger.writeString("state", state.toString());
            }),
            new SysIdRoutine.Mechanism(v -> {
                io.setMasterRequest(new VoltageOut(v));

                io.updateInputs(inputs);
                Logger.processInputs("Shooter", inputs);
            }, null, this));
    }

    public void operate(ShooterState state) {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);

        if(Constants.GetTuningMode() == TuningMode.ACTIVE) setMotorConfigs();

        double Entry_Power = ShooterConstants.ENTRY_POWER_STOP;
        double Angle_Position = calculateShooterPosition();
        double Shoot_Power = ShooterTuning.Shoot_Voltage_Idle_get();
        
        switch(state) {
            case INTAKE_HELP:
                Entry_Power = ShooterTuning.Entry_Power_Roll_get();
                Shoot_Power = ShooterTuning.Shoot_Voltage_Help_get();
                break;
                
            case SPEED_UP:
                Shoot_Power = ShooterTuning.Shoot_Voltage_Fire_get();
                break;

            case SHOOT:
                Entry_Power = ShooterTuning.Entry_Power_Roll_get();
                Shoot_Power = ShooterTuning.Shoot_Voltage_Fire_get();
                break;

            default:
            case IDLE:
                // Do nothing
                break;
        }

        io.setEntryRequest(new DutyCycleOut(Entry_Power));
        io.setAngleRequest(new MotionMagicVoltage(Angle_Position));
        io.setMasterRequest(new VoltageOut(Shoot_Power));
    }

    public ShooterIOInputs getInputs() {
        return inputs;
    }

    private double calculateShooterPosition() {
        if(Dashboard.getAcceptAngleChanges()) return Dashboard.getAngleValue();
        else return 0; // TO-DO
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

        talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        talonFXConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // Inverted
        talonFXConfigs.CurrentLimits.StatorCurrentLimit = ShooterConstants.Entry_StatorCurrentLimit;
        talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        talonFXConfigs.CurrentLimits.SupplyCurrentLimit = ShooterConstants.Entry_SupplyCurrentLimit;
        talonFXConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

        io.applyEntryConfiguration(talonFXConfigs);
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
        motionMagicConfigs.MotionMagicAcceleration = ShooterTuning.Angle_Expo_kA_get();
        motionMagicConfigs.MotionMagicExpo_kV = ShooterTuning.Angle_Expo_kV_get();
        // motionMagicConfigs.MotionMagicExpo_kA = ShooterTuning.Angle_Expo_kA_get();

        talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        talonFXConfigs.CurrentLimits.StatorCurrentLimit = ShooterConstants.Angle_StatorCurrentLimit;
        talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        talonFXConfigs.CurrentLimits.SupplyCurrentLimit = ShooterConstants.Angle_SupplyCurrentLimit;
        talonFXConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

        io.applyAngleConfiguration(talonFXConfigs);
    }

    private void setShootMotorsConfigs() {
        io.setSlaveRequest(new Follower(ShooterConstants.ShooterMasterMotorID, MotorAlignmentValue.Opposed));

        TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
        Slot0Configs slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = ShooterTuning.Shoot_kS_get();
        slot0Configs.kV = ShooterTuning.Shoot_kV_get();
        slot0Configs.kA = ShooterTuning.Shoot_kA_get();
        slot0Configs.kP = ShooterTuning.Shoot_kP_get();
        slot0Configs.kI = ShooterTuning.Shoot_kI_get();
        slot0Configs.kD = ShooterTuning.Shoot_kD_get();

        talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        talonFXConfigs.CurrentLimits.StatorCurrentLimit = ShooterConstants.Shoot_StatorCurrentLimit;
        talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        talonFXConfigs.CurrentLimits.SupplyCurrentLimit = ShooterConstants.Shoot_SupplyCurrentLimit;
        talonFXConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

        io.applyMasterConfiguration(talonFXConfigs);

        TalonFXConfiguration slaveTalonFXConfigs = new TalonFXConfiguration();

        slaveTalonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        slaveTalonFXConfigs.CurrentLimits.StatorCurrentLimit = ShooterConstants.Shoot_StatorCurrentLimit;
        slaveTalonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        slaveTalonFXConfigs.CurrentLimits.SupplyCurrentLimit = ShooterConstants.Shoot_SupplyCurrentLimit;
        slaveTalonFXConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

        io.applySlaveConfiguration(slaveTalonFXConfigs);
    }

    private void setMotorConfigs() {
        setEntryMotorConfigs();
        setAngleMotorConfigs();
        setShootMotorsConfigs();
    }

    public void simulationPeriodic() {
        io.simulationPeriodic();
    }

    // SysId routines //

    private SysIdRoutine Entry_SysIdRoutine;
    private SysIdRoutine Angle_SysIdRoutine;
    private SysIdRoutine Shoot_SysIdRoutine;

    public Command Entry_SysIdQuasistatic(SysIdRoutine.Direction direction) {
        return Entry_SysIdRoutine.quasistatic(direction);
    }

    public Command Entry_SysIdDynamic(SysIdRoutine.Direction direction) {
        return Entry_SysIdRoutine.dynamic(direction);
    }

    public Command Angle_SysIdQuasistatic(SysIdRoutine.Direction direction) {
        return Angle_SysIdRoutine.quasistatic(direction);
    }

    public Command Angle_SysIdDynamic(SysIdRoutine.Direction direction) {
        return Angle_SysIdRoutine.dynamic(direction);
    }

    public Command Shoot_SysIdQuasistatic(SysIdRoutine.Direction direction) {
        return Shoot_SysIdRoutine.quasistatic(direction);
    }

    public Command Shoot_SysIdDynamic(SysIdRoutine.Direction direction) {
        return Shoot_SysIdRoutine.dynamic(direction);
    }
}
