package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.IO.IntakeIO;
import frc.robot.subsystems.IO.IntakeIO.IntakeIOInputs;

public class IntakeSubsystem implements Subsystem {
    private IntakeIO io;
    private IntakeIOInputs inputs = new IntakeIOInputs();

    public IntakeSubsystem(IntakeIO _io) {        
        io = _io;

        setMotorConfigs();

        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);

        Intake_SysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.per(Second).of(IntakeConstants.Intake_SysIdQuasistatic), 
                Volts.of(IntakeConstants.Intake_SysIdDynamic), null, state -> {
                    SignalLogger.writeString("state", state.toString());
            }),
            new SysIdRoutine.Mechanism(v -> io.setIntakeRequest(new VoltageOut(v)), null, this));

        Control_SysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.per(Second).of(IntakeConstants.Control_SysIdQuasistatic), 
                Volts.of(IntakeConstants.Control_SysIdDynamic), null, state -> {
                    SignalLogger.writeString("state", state.toString());
            }),
            new SysIdRoutine.Mechanism(v -> io.setControlRequest(new VoltageOut(v)), null, this));
    }

    public void operate(IntakeState state, StorageState storageState) {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);

        double Power = IntakeConstants.INTAKE_POWER_STOP;
        double Position = IntakeConstants.CONTROL_POS_OUT;

        switch (state) {
            case ACTIVE:
                Power = IntakeTuning.Power_Intake_get();
                break;

            case DEPLETE:
                Power = IntakeTuning.Power_Deplete_get();
                break;

            default:
            case INACTIVE:
                // Do nothing
                break;
        }

        switch(storageState) {
            case IN:
                Position = IntakeConstants.CONTROL_POS_IN;

            default:
            case OUT:
                Position = IntakeConstants.CONTROL_POS_OUT;
        }

        if(Math.abs(inputs.Control_Position - Position) > IntakeConstants.CONTROL_POS_TOLERANCE ||
            Position == IntakeConstants.CONTROL_POS_IN) {
            io.setControlRequest(new MotionMagicExpoVoltage(Position));
        } else {
            io.setControlRequest(new CoastOut());
        }

        io.setIntakeRequest(new DutyCycleOut(Power));
    }

    private void setIntakeMotorConfigs() {
        TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
        Slot0Configs slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = IntakeTuning.Intake_kS_get();
        slot0Configs.kV = IntakeTuning.Intake_kV_get();
        slot0Configs.kA = IntakeTuning.Intake_kA_get();
        slot0Configs.kP = IntakeTuning.Intake_kP_get();
        slot0Configs.kI = IntakeTuning.Intake_kI_get();
        slot0Configs.kD = IntakeTuning.Intake_kD_get();

        talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        talonFXConfigs.CurrentLimits.StatorCurrentLimit = IntakeConstants.Intake_StatorCurrentLimit;
        talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        talonFXConfigs.CurrentLimits.SupplyCurrentLimit = IntakeConstants.Intake_SupplyCurrentLimit;
        talonFXConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

        io.applyIntakeConfiguration(talonFXConfigs);
    }

    private void setControlMotorConfigs() {
        TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
        Slot0Configs slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = IntakeTuning.Control_kS_get();
        slot0Configs.kV = IntakeTuning.Control_kV_get();
        slot0Configs.kA = IntakeTuning.Control_kA_get();
        slot0Configs.kP = IntakeTuning.Control_kP_get();
        slot0Configs.kI = IntakeTuning.Control_kI_get();
        slot0Configs.kD = IntakeTuning.Control_kD_get();

        MotionMagicConfigs motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = IntakeTuning.Control_Expo_CV_get();
        motionMagicConfigs.MotionMagicExpo_kV = IntakeTuning.Control_Expo_kV_get();
        motionMagicConfigs.MotionMagicExpo_kA = IntakeTuning.Control_Expo_kA_get();

        talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        talonFXConfigs.CurrentLimits.StatorCurrentLimit = IntakeConstants.Control_StatorCurrentLimit;
        talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        talonFXConfigs.CurrentLimits.SupplyCurrentLimit = IntakeConstants.Control_SupplyCurrentLimit;

        io.applyControlConfiguration(talonFXConfigs);
    }

    private void setMotorConfigs() {
        setIntakeMotorConfigs();
        setControlMotorConfigs();
    }

    public void simulationPeriodic() {
        io.simulationPeriodic();
    }

    // SysId routines //

    private SysIdRoutine Intake_SysIdRoutine;    
    private SysIdRoutine Control_SysIdRoutine;    

    public Command Intake_SysIdQuasistatic(SysIdRoutine.Direction direction) {
        return Intake_SysIdRoutine.quasistatic(direction);
    }

    public Command Intake_SysIdDynamic(SysIdRoutine.Direction direction) {
        return Intake_SysIdRoutine.dynamic(direction);
    }

    public Command Control_SysIdQuasistatic(SysIdRoutine.Direction direction) {
        return Control_SysIdRoutine.quasistatic(direction);
    }

    public Command Control_SysIdDynamic(SysIdRoutine.Direction direction) {
        return Control_SysIdRoutine.dynamic(direction);
    }
}