package frc.robot.subsystems.IO.Stub;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;

import frc.robot.subsystems.IO.IntakeIO;

public class IntakeStub implements IntakeIO {
    public IntakeStub() {}

    public void updateInputs(IntakeIOInputs inputs) {}

    public void applyIntakeConfiguration(TalonFXConfiguration configuration) {}

    public void applyControlConfiguration(TalonFXConfiguration configuration) {}

    public void setControlPosition(double newValue) {}

    public void setIntakeRequest(ControlRequest request) {}

    public void setControlRequest(ControlRequest request) {}

    public void simulationPeriodic() {}

    @Override
    public void close() throws Exception {}
}