package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj.Timer;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
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
    // HELP_SHOOT toggle state
    private boolean helpShootOpen = false;
    private double lastHelpToggleTime = 0.0;

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
                Intake_Power = IntakeTuning.POWER_INTAKE_get();
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
                Control_Power = IntakeTuning.CONTROL_POWER_CLOSE_get();
                break;
                
            case HELP_SHOOT:
                // Toggle open/close on a fixed time interval. Position is ignored.
                double now = Timer.getFPGATimestamp();
                if (now - lastHelpToggleTime > IntakeConstants.HELP_TOGGLE_PERIOD) {
                    helpShootOpen = !helpShootOpen;
                    lastHelpToggleTime = now;
                }

                if (helpShootOpen) {
                    Control_Power = IntakeTuning.CONTROL_POWER_SHAKE_get();
                } else {
                    Control_Power = -IntakeTuning.CONTROL_POWER_SHAKE_get();
                }
                break;

            default:
            case OUT:
                Position = IntakeConstants.CONTROL_POS_OUT;
                Control_Power = IntakeTuning.CONTROL_POWER_OPEN_get();
                break;

        }

        // Use Motion Magic to move the control motor to the requested position when not
        // in HELP_SHOOT. Keep the existing HELP_SHOOT shake behavior, and keep coast
        // when the control is open (OUT).
        if(storageState != StorageState.HELP_SHOOT) {
            if(Math.abs(inputs.Control_Position - Position) > IntakeConstants.CONTROL_POS_TOLERANCE) {
                // Command the target position using Motion Magic (closed-loop motion profiling)
                io.setControlRequest(new MotionMagicVoltage(Position));
            // } else if(storageState == StorageState.IN) {
            //     // Hold/stopped when fully in
            //     io.setControlRequest(new DutyCycleOut(IntakeConstants.CONTROL_POWER_STOP));
            } 
            else {
                // When open/out, let the mechanism coast
                io.setControlRequest(new CoastOut());
            }
        } else {
            // HELP_SHOOT keeps the existing shake duty-cycle behaviour
            io.setControlRequest(new DutyCycleOut(Control_Power));
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
        talonFXConfigs.CurrentLimits.StatorCurrentLimit = IntakeConstants.Control_StatorCurrentLimit;
        talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        talonFXConfigs.CurrentLimits.SupplyCurrentLimit = IntakeConstants.Control_SupplyCurrentLimit;

    MotionMagicConfigs motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = IntakeTuning.CONTROL_CruiseVelocity_get();
    motionMagicConfigs.MotionMagicAcceleration = IntakeTuning.CONTROL_Acceleration_get();
    motionMagicConfigs.MotionMagicJerk = IntakeTuning.CONTROL_Jerk_get();

    // Slot0 PID (kP) for closed-loop position control used by Motion Magic
    Slot0Configs slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kP = IntakeTuning.CONTROL_kP_get();
    slot0Configs.kV = IntakeTuning.CONTROL_kV_get();

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