package frc.robot.subsystems.Delivery.Conveyor;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
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

        sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.per(Second).of(ConveyorConstants.SysIdQuasistatic), 
                Volts.of(ConveyorConstants.SysIdDynamic), null, state -> {
                    SignalLogger.writeString("state", state.toString());
            }),
            new SysIdRoutine.Mechanism(v -> {
                io.setRequest(new VoltageOut(v));

                io.updateInputs(inputs);
                Logger.processInputs("Shooter", inputs);
            }, null, this));
    }

    public void operate(ConveyorState state) {
        io.updateInputs(inputs);
        Logger.processInputs("Conveyor", inputs);

        if(Constants.GetTuningMode() == TuningMode.ACTIVE) setMotorConfigs();

        double Power = ConveyorConstants.CONVEYOR_POWER_STOP;

        switch(state) {
            case GUIDE:
                Power = ConveyorTuning.Power_Guide_get();
                break;

            case DEPLETE:
                Power = ConveyorTuning.Power_Deplete_get();
                break;

            case UNJAM:
                double modulo = (1000 * Timer.getFPGATimestamp()) % (2 * ConveyorTuning.Unjam_Half_Cycle_Length_get());
                if(modulo > ConveyorTuning.Unjam_Half_Cycle_Length_get()) {
                    Power = ConveyorTuning.Power_Guide_get();
                } else {
                    Power = ConveyorTuning.Power_Deplete_get();
                }
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
        Slot0Configs slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = ConveyorTuning.kS_get();
        slot0Configs.kV = ConveyorTuning.kV_get();
        slot0Configs.kA = ConveyorTuning.kA_get();
        slot0Configs.kP = ConveyorTuning.kP_get();
        slot0Configs.kI = ConveyorTuning.kI_get();
        slot0Configs.kD = ConveyorTuning.kD_get();

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

    // SysId routines //

    private SysIdRoutine sysIdRoutine;
    
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }
}
