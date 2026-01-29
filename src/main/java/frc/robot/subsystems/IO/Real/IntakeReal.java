package frc.robot.subsystems.IO.Real;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.Intake.IntakeConstants;
import frc.robot.subsystems.IO.IntakeIO;

public class IntakeReal implements IntakeIO {
    private final TalonFX master;

    public IntakeReal() {        
        master = new TalonFX(IntakeConstants.IntakeMotorID);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.position = master.getPosition().getValueAsDouble();
        inputs.velocity = master.getVelocity().getValueAsDouble();
        inputs.appliedVolts = master.getMotorVoltage().getValueAsDouble();
        inputs.currentAmps = master.getStatorCurrent().getValueAsDouble();
    }

    @Override
    public void applyTalonFXConfig(TalonFXConfiguration configuration) {
        master.getConfigurator().apply(configuration);
    }

    @Override
    public void setPosition(double newValue) {
        master.setPosition(newValue);
    }

    @Override
    public void setMotionMagic(DutyCycleOut request) {
        master.setControl(request);
    }

    @Override
    public void setVoltage(Voltage voltage) {
        master.setVoltage(voltage.in(Volts));
    }

    @Override
    public void simulationPeriodic() {}

    @Override
    public void close() throws Exception {
        master.close();
    }
}