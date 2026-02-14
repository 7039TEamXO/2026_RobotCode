package frc.robot.subsystems.IO.Real;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.subsystems.Delivery.Conveyor.ConveyorConstants;
import frc.robot.subsystems.IO.ConveyorIO;

public class ConveyorReal implements ConveyorIO {
    private final TalonFX motor;

    public ConveyorReal() {
        motor = new TalonFX(ConveyorConstants.ConveyorMotorID);
    }

    public void updateInputs(ConveyorIOInputs inputs) {
        inputs.Position = motor.getPosition().getValueAsDouble();
        inputs.Velocity = motor.getVelocity().getValueAsDouble();
        inputs.AppliedVolts = motor.getMotorVoltage().getValueAsDouble();
        inputs.CurrentAmps = motor.getStatorCurrent().getValueAsDouble();
    }

    public void applyConfiguration(TalonFXConfiguration configuration) {
        motor.getConfigurator().apply(configuration);
    }

    public void setPosition(double newValue) {
        motor.setPosition(newValue);
    }

    public void setRequest(ControlRequest request) {
        motor.setControl(request);
    }

    public void simulationPeriodic() {}

    @Override
    public void close() throws Exception {
        motor.close();
    }
}