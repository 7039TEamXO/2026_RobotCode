package frc.robot.subsystems.IO.Real;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.subsystems.Delivery.Conveyor.ConveyorConstants;
import frc.robot.subsystems.IO.ConveyorIO;

public class ConveyorReal implements ConveyorIO {
    private final TalonFX masterMotor;

    public ConveyorReal() {
        masterMotor = new TalonFX(ConveyorConstants.ConveyorMotorID);
    }

    public void updateInputs(ConveyorIOInputs inputs) {
        inputs.Position = masterMotor.getPosition().getValueAsDouble();
        inputs.Velocity = masterMotor.getVelocity().getValueAsDouble();
        inputs.AppliedVolts = masterMotor.getMotorVoltage().getValueAsDouble();
        inputs.CurrentAmps = masterMotor.getStatorCurrent().getValueAsDouble();
    }

    public void applyConfiguration(TalonFXConfiguration configuration) {
        masterMotor.getConfigurator().apply(configuration);
    }

    public void setPosition(double newValue) {
        masterMotor.setPosition(newValue);
    }

    public void setRequest(ControlRequest request) {
        masterMotor.setControl(request);
    }

    public void simulationPeriodic() {}

    @Override
    public void close() throws Exception {
        masterMotor.close();
    }
}