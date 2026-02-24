package frc.robot.subsystems.Delivery.Conveyor;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Constants.TuningMode;
import frc.robot.subsystems.IO.ConveyorIO;
import frc.robot.subsystems.IO.ConveyorIO.ConveyorIOInputs;

public class ConveyorSubsystem implements Subsystem {
    private final ConveyorIO io;
    private final ConveyorIOInputs inputs;

    public ConveyorSubsystem(ConveyorIO _io) {
        io = _io;
        inputs = new ConveyorIOInputs();

        setMotorConfigs();

        io.updateInputs(inputs);
        Logger.processInputs("Conveyor", inputs);
    }

    public void operate(ConveyorState state) {
        io.updateInputs(inputs);
        Logger.processInputs("Conveyor", inputs);
        if(Constants.GetTuningMode() == TuningMode.ACTIVE) setMotorConfigs();
        double Power = ConveyorConstants.CONVEYOR_POWER_STOP;

        switch(state) {
            case GUIDE:
                Power = ConveyorTuning.POWER_GUIDE_get();
                break;

            case DEPLETE:
                Power = ConveyorTuning.POWER_DEPLETE_get();
                break;

            case UNJAM:
                double modulo = (1000 * Timer.getFPGATimestamp()) % (2 * ConveyorTuning.UNJAM_HALF_CYCLE_LENGTH_get());
                if(modulo > ConveyorTuning.UNJAM_HALF_CYCLE_LENGTH_get()) {
                    Power = ConveyorTuning.POWER_GUIDE_get();
                } else {
                    Power = ConveyorTuning.POWER_DEPLETE_get();
                }
                break;

            case INTAKE_HELP:
                Power = ConveyorTuning.POWER_INTAKE_HELP_get();
                break;

            default:
            case STOP:
                // Do nothing
                break;
        }

        io.setRequest(new DutyCycleOut(Power));
    }

    private void setMotorConfigs() {
        TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();

        talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        talonFXConfigs.CurrentLimits.StatorCurrentLimit = ConveyorConstants.StatorCurrentLimit;
        talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        talonFXConfigs.CurrentLimits.SupplyCurrentLimit = ConveyorConstants.SupplyCurrentLimit;
        talonFXConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

        io.applyConfiguration(talonFXConfigs);
    }

    public void simulationPeriodic() {
        io.simulationPeriodic();
    }
}
