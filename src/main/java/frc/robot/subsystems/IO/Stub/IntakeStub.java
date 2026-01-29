package frc.robot.subsystems.IO.Stub;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.IO.IntakeIO;

public class IntakeStub implements IntakeIO {
    public IntakeStub() {}

    @Override
    public void updateInputs(IntakeIOInputs inputs) {}

    @Override
    public void applyIntakeTalonFXConfig(TalonFXConfiguration configuration) {}

    @Override
    public void applyControlTalonFXConfig(TalonFXConfiguration configuration) {}

    @Override
    public void setIntakePosition(double newValue) {}

    @Override
    public void setControlPosition(double newValue) {}

    @Override
    public void setIntakeMotionMagic(DutyCycleOut request) {}

    @Override
    public void setControlMotionMagic(DutyCycleOut request) {}

    @Override
    public void setIntakeVoltage(Voltage voltage) {}

    @Override
    public void setControlVoltage(Voltage voltage) {}

    @Override
    public void simulationPeriodic() {}

    @Override
    public void close() throws Exception {}
}