package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.TuningMode;
import frc.robot.subsystems.IO.IntakeIO;
import frc.robot.subsystems.IO.IntakeIO.IntakeIOInputs;

public class IntakeSubsystem implements Subsystem {
    private IntakeIO io;
    private IntakeIOInputs inputs = new IntakeIOInputs();

    public IntakeSubsystem(IntakeIO _io) {        
        io = _io;

        setMotorConfigs();
        io.setControlPosition(0);

        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
        Control_SysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.per(Second).of(IntakeConstants.Control_SysIdQuasistatic),
                Volts.of(IntakeConstants.Control_SysIdDynamic), null, state -> {
                    SignalLogger.writeString("state", state.toString());
                }), 
            new SysIdRoutine.Mechanism(v -> {
                io.setControlRequest(new VoltageOut(v));

                io.updateInputs(inputs);
                Logger.processInputs("Intake", inputs);
            }, null, this));
    }

    public void operate(IntakeState state, StorageState storageState) {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);

        if(Constants.GetTuningMode() == TuningMode.ACTIVE) setMotorConfigs();

        double Intake_Power = IntakeConstants.INTAKE_POWER_STOP;
        double Position = IntakeConstants.CONTROL_POS_OUT;
        double Control_Power = IntakeConstants.CONTROL_POWER_STOP;

        switch (state) {
            case ACTIVE:
                Intake_Power = IntakeTuning.Power_Intake_get();
                break;

            case DEPLETE:
                Intake_Power = IntakeTuning.Power_Deplete_get();
                break;

            default:
            case INACTIVE:
                // Do nothing
                break;
        }

        switch(storageState) {
            case IN:
                Position = IntakeConstants.CONTROL_POS_IN;
                Control_Power = IntakeConstants.CONTROL_POWER_CLOSE;
                break;

            default:
            case OUT:
                Position = IntakeConstants.CONTROL_POS_OUT;
                Control_Power = IntakeConstants.CONTROL_POWER_OPEN;
                break;
        }

        if(Math.abs(inputs.Control_Position - Position) > IntakeConstants.CONTROL_POS_TOLERANCE) {
            io.setControlRequest(new DutyCycleOut(Control_Power));
        } else if(storageState == StorageState.IN) {
            io.setControlRequest(new DutyCycleOut(IntakeConstants.CONTROL_POWER_STOP));
        } else {
            io.setControlRequest(new CoastOut());
        }

        io.setIntakeRequest(new DutyCycleOut(Intake_Power));
    }

    private void setIntakeMotorConfigs() {
        TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();

        talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        talonFXConfigs.CurrentLimits.StatorCurrentLimit = IntakeConstants.Intake_StatorCurrentLimit;
        talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        talonFXConfigs.CurrentLimits.SupplyCurrentLimit = IntakeConstants.Intake_SupplyCurrentLimit;
        talonFXConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

        io.applyIntakeConfiguration(talonFXConfigs);
    }

    private void setControlMotorConfigs() {
        TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();

        talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        talonFXConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // Inverted
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

    private SysIdRoutine Control_SysIdRoutine;    

    public Command Control_SysIdQuasistatic(SysIdRoutine.Direction direction) {
        return Control_SysIdRoutine.quasistatic(direction);
    }

    public Command Control_SysIdDynamic(SysIdRoutine.Direction direction) {
        return Control_SysIdRoutine.dynamic(direction);
    }
}