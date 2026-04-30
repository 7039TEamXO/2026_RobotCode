package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.wpilibj.Timer;
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

    private boolean helpShootOpen = false;
    private double lastHelpToggleTime = 0.0;

    // track when we entered HELP_SHOOT so we can wait
    private double helpShootEntryTime = 0.0;

    // track previous storage state for entry detection
    private StorageState lastStorageState = StorageState.OUT;

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
                if(inputs.Intake_Velocity < 75){
                Intake_Power = IntakeConstants.INTAKE_CURRENT_KICK;
                }
        
             else {
                Intake_Power = (IntakeConstants.INTAKE_CURRENT_FREE);
                }
                break;

            case HELP_SHOOT:
                Intake_Power = IntakeTuning.POWER_HELP_SHOOT_get();
                break;

            case DEPLETE:
                Intake_Power = IntakeTuning.POWER_DEPLETE_get();
                break;

            default:
            case INACTIVE:
                // Do nothing
                break;
        }

        switch(storageState) {
            case IN:
                Position = IntakeConstants.CONTROL_POS_IN;
                break;
                
            case HELP_SHOOT:
                // On entry into HELP_SHOOT, record the time so we can wait a fixed delay
                double now = Timer.getFPGATimestamp();
                if (storageState != lastStorageState) {
                    helpShootEntryTime = now;
                    lastHelpToggleTime = now;
                    helpShootOpen = false;
                }

                // During the initial delay keep the control arm out and don't pump
                if (now - helpShootEntryTime > IntakeConstants.INITIAL_STOP_TIME) {
                    // After delay, toggle open/close on configured period
                    if (now - lastHelpToggleTime > IntakeConstants.HELP_TOGGLE_PERIOD) {
                        helpShootOpen = !helpShootOpen;
                        lastHelpToggleTime = now;
                    }

                    if (helpShootOpen) {
                        Control_Power = IntakeTuning.CONTROL_POWER_SHAKE_get();
                    } else {
                        Control_Power = -IntakeTuning.CONTROL_POWER_SHAKE_get();
                    }
                } else {
                    lastHelpToggleTime = now;
                }

                break;

            default:
            case OUT:
                Position = IntakeConstants.CONTROL_POS_OUT;
                break;

        }

        if(storageState != StorageState.HELP_SHOOT) {
            if(Math.abs(inputs.Control_Position - Position) > IntakeConstants.CONTROL_POS_TOLERANCE) {
                io.setControlRequest(new MotionMagicVoltage(Position).withEnableFOC(true)); //TODO: FOC
            } else {
                io.setControlRequest(new CoastOut());
            }
        } else {
            io.setControlRequest(new DutyCycleOut(Control_Power).withEnableFOC(true)); //TODO: FOC
        }
        
            io.setIntakeRequest(new TorqueCurrentFOC(Intake_Power));
        // io.setIntakeRequest(new DutyCycleOut(Intake_Power).withEnableFOC(true)); //TODO: FOC
        
        lastStorageState = storageState;
    }

    private void setIntakeMotorConfigs() {
        TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();

        talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        talonFXConfigs.CurrentLimits.StatorCurrentLimit = IntakeConstants.Intake_StatorCurrentLimit;
        talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        talonFXConfigs.CurrentLimits.SupplyCurrentLimit = IntakeConstants.Intake_SupplyCurrentLimit;
        talonFXConfigs.CurrentLimits.SupplyCurrentLowerLimit = 90;
        talonFXConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        talonFXConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        io.applyIntakeConfiguration(talonFXConfigs);
    }

    private void setControlMotorConfigs() {
        TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();

        talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        talonFXConfigs.CurrentLimits.StatorCurrentLimit = IntakeConstants.Control_StatorCurrentLimit;
        talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        talonFXConfigs.CurrentLimits.SupplyCurrentLimit = IntakeConstants.Control_SupplyCurrentLimit;
        talonFXConfigs.CurrentLimits.SupplyCurrentLowerLimit = 90;
        talonFXConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

        MotionMagicConfigs motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = IntakeTuning.Control_CruiseVelocity_get();
        motionMagicConfigs.MotionMagicAcceleration = IntakeTuning.Control_Acceleration_get();

        Slot0Configs slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = IntakeTuning.Control_kS_get();
        slot0Configs.kV = IntakeTuning.Control_kV_get();
        slot0Configs.kA = IntakeTuning.Control_kA_get();
        slot0Configs.kP = IntakeTuning.Control_kP_get();
        slot0Configs.kI = IntakeTuning.Control_kI_get();
        slot0Configs.kD = IntakeTuning.Control_kD_get();
        slot0Configs.kG = IntakeTuning.Control_kG_get();

        slot0Configs.GravityType = GravityTypeValue.Elevator_Static;

        io.applyControlConfiguration(talonFXConfigs);
    }

    public void resetControlEncoder() {
        if(lastStorageState == StorageState.IN) {
            io.setControlPosition(IntakeConstants.CONTROL_POS_IN);
        } else if(lastStorageState == StorageState.OUT) {
            io.setControlPosition(IntakeConstants.CONTROL_POS_OUT);
        }
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