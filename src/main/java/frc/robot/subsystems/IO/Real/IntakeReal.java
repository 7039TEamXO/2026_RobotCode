package frc.robot.subsystems.IO.Real;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.Intake.IntakeConstants;
import frc.robot.subsystems.IO.IntakeIO;

public class IntakeReal implements IntakeIO {
    private final TalonFX intakeMotor;
    private final TalonFX controlMotor;

    public IntakeReal() {        
        intakeMotor = new TalonFX(IntakeConstants.IntakeMotorID);
        controlMotor = new TalonFX(IntakeConstants.ControlMotorID);
    }

    public void updateInputs(IntakeIOInputs inputs) {
        inputs.Intake_Position = intakeMotor.getPosition().getValueAsDouble();
        inputs.Intake_Velocity = intakeMotor.getVelocity().getValueAsDouble();
        inputs.Intake_AppliedVolts = intakeMotor.getMotorVoltage().getValueAsDouble();
        inputs.Intake_CurrentAmps = intakeMotor.getStatorCurrent().getValueAsDouble();
    }

    public void applyIntakeTalonFXConfig(TalonFXConfiguration configuration) {
        intakeMotor.getConfigurator().apply(configuration);
    }

    public void applyControlTalonFXConfig(TalonFXConfiguration configuration) {
        controlMotor.getConfigurator().apply(configuration);
    }

    public void setIntakePosition(double newValue) {
        intakeMotor.setPosition(newValue);
    }

    public void setControlPosition(double newValue) {
        controlMotor.setPosition(newValue);
    }

    public void setIntakeMotionMagic(DutyCycleOut request) {
        intakeMotor.setControl(request);
    }

    public void setControlMotionMagic(DutyCycleOut request) {
        controlMotor.setControl(request);
    }

    public void setIntakeVoltage(Voltage voltage) {
        intakeMotor.setVoltage(voltage.in(Volts));
    }

    public void setControlVoltage(Voltage voltage) {
        controlMotor.setVoltage(voltage.in(Volts));
    }

    public void simulationPeriodic() {}

    @Override
    public void close() throws Exception {
        intakeMotor.close();
        controlMotor.close();
    }
}