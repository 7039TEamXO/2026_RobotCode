package frc.robot.subsystems.IO.Stub;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;

import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.IO.ShooterIO;

public class ShooterStub implements ShooterIO {
    public ShooterStub() {}

    public void updateInputs(ShooterIOInputs inputs) {}

    public void applyEntryTalonFXConfig(TalonFXConfiguration configuration) {}

    public void applyAngleTalonFXConfig(TalonFXConfiguration configuration) {}

    public void applyMasterTalonFXConfig(TalonFXConfiguration configuration) {}

    public void applySlaveTalonFXConfig(TalonFXConfiguration configuration) {}

    public void setAngleMotorPosition(double newValue) {}

    public void setEntryMotionMagic(DutyCycleOut request) {}

    public void setAngleMotionMagic(MotionMagicExpoVoltage request) {}

    public void setMasterMotionMagic(DutyCycleOut request) {}

    public void setSlaveMotionMagic(Follower request) {}

    public void setEntryMotorVoltage(Voltage voltage) {}

    public void setAngleMotorVoltage(Voltage voltage) {}

    public void setMasterMotorVoltage(Voltage voltage) {}

    public void setSlaveMotorVoltage(Voltage voltage) {}

    public void simulationPeriodic() {}

    @Override
    public void close() throws Exception {}
}
