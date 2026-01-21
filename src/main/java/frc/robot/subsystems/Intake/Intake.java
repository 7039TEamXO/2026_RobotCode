package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.subsystems.IO.IntakeIO;
import frc.robot.subsystems.IO.IntakeIO.IntakeIOInputs;

public class Intake implements Subsystem {
    private IntakeIO io;
    private IntakeIOInputs inputs = new IntakeIOInputs();
      
    private double power = IntakeConstants.INTAKE_POWER_STOP;

    public Intake(IntakeIO _io) {        
        io = _io;

        var talonFXConfigs = new TalonFXConfiguration();
        talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        io.applyTalonFXConfig(talonFXConfigs);

        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);

        sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.per(Second).of(IntakeConstants.SysIdQuasistatic), 
                Volts.of(IntakeConstants.SysIdDynamic), null, state -> {
                    SignalLogger.writeString("state", state.toString());
            }),
            new SysIdRoutine.Mechanism(v -> io.setVoltage(v), null, this));
    }

    public void operate(IntakeState state) {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);

        switch (state) {
            case ACTIVE:
                power = IntakeConstants.INTAKE_POWER_INTAKE;
                break;

            case DEPLETE:
                power = IntakeConstants.INTAKE_POWER_DEPLETE;
                break;
            
            case INACTIVE:
                power = IntakeConstants.INTAKE_POWER_STOP;
                break;
        }

        io.setMotionMagic(new DutyCycleOut(power));
    }

    public double getHandlerMotorDistance() {
        return inputs.position;
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