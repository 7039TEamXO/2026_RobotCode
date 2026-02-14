package frc.robot.subsystems.IO.Stub;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;

import frc.robot.subsystems.IO.ShooterIO;

public class ShooterStub implements ShooterIO {
    public ShooterStub() {}

    public void updateInputs(ShooterIOInputs inputs) {}

    public void applyEntryConfiguration(TalonFXConfiguration configuration) {}

    public void applyAngleConfiguration(TalonFXConfiguration configuration) {}

    public void applyMasterConfiguration(TalonFXConfiguration configuration) {}

    public void applySlaveConfiguration(TalonFXConfiguration configuration) {}

    public void setAngleMotorPosition(double newValue) {}

    public void setEntryRequest(ControlRequest request) {}

    public void setAngleRequest(ControlRequest request) {}

    public void setMasterRequest(ControlRequest request) {}

    public void setSlaveRequest(ControlRequest request) {}

    public void simulationPeriodic() {}

    @Override
    public void close() throws Exception {}
}
