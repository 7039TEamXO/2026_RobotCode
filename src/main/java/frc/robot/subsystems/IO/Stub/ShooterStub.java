package frc.robot.subsystems.IO.Stub;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.IO.ShooterIO;

public class ShooterStub implements ShooterIO {
    public ShooterStub() {}

    public void updateInputs(ShooterIOInputs inputs) {}

    public void applyEntryTalonFXConfig(TalonFXConfiguration configuration) {}

    public void applyAngleTalonFXConfig(TalonFXConfiguration configuration) {}

    public void applyMasterTalonFXConfig(TalonFXConfiguration configuration) {}

    public void applySlaveTalonFXConfig(TalonFXConfiguration configuration) {}

    public void setEntryMotorPosition(double newValue) {}

    public void setAngleMotorPosition(double newValue) {}

    public void setMasterMotorPosition(double newValue) {}

    public void setSlaveMotorPosition(double newValue) {}

    public void setEntryMotionMagic(MotionMagicExpoVoltage request) {}

    public void setAngleMotionMagic(MotionMagicExpoVoltage request) {}

    public void setMasterMotionMagic(MotionMagicExpoVoltage request) {}

    public Follower createMasterFollower() {
        return new Follower(0, MotorAlignmentValue.Opposed);
    }

    public void setSlaveMotionMagic(Follower request) {}

    public void setEntryMotorVoltage(Voltage voltage) {}

    public void setAngleMotorVoltage(Voltage voltage) {}

    public void setMasterMotorVoltage(Voltage voltage) {}

    public void setSlaveMotorVoltage(Voltage voltage) {}

    public void simulationPeriodic() {}

    @Override
    public void close() throws Exception {}
}
