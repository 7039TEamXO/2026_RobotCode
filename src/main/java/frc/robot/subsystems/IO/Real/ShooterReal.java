package frc.robot.subsystems.IO.Real;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.subsystems.Delivery.Shooter.ShooterConstants;
import frc.robot.subsystems.IO.ShooterIO;

public class ShooterReal implements ShooterIO {
    private final TalonFX entryMotor;
    private final TalonFX angleMotor;
    private final TalonFX masterMotor;
    private final TalonFX slaveMotor;

    private final DutyCycleEncoder absoluteEncoder;

    public ShooterReal() {
        entryMotor = new TalonFX(ShooterConstants.ShooterEntryMotorID);
        angleMotor = new TalonFX(ShooterConstants.ShooterAngleMotorID);
        masterMotor = new TalonFX(ShooterConstants.ShooterMasterMotorID);
        slaveMotor = new TalonFX(ShooterConstants.ShooterSlaveMotorID);

        absoluteEncoder = new DutyCycleEncoder(ShooterConstants.AbsoluteEncoderID);
    }

    public void updateInputs(ShooterIOInputs inputs) {
        inputs.EntryMotor_Position = entryMotor.getPosition().getValueAsDouble();
        inputs.EntryMotor_Velocity = entryMotor.getVelocity().getValueAsDouble();
        inputs.EntryMotor_AppliedVolts = entryMotor.getMotorVoltage().getValueAsDouble();
        inputs.EntryMotor_CurrentAmps = entryMotor.getStatorCurrent().getValueAsDouble();

        inputs.AngleMotor_Position = angleMotor.getPosition().getValueAsDouble();
        inputs.AngleMotor_Velocity = angleMotor.getVelocity().getValueAsDouble();
        inputs.AngleMotor_AppliedVolts = angleMotor.getMotorVoltage().getValueAsDouble();
        inputs.AngleMotor_CurrentAmps = angleMotor.getStatorCurrent().getValueAsDouble();

        inputs.ShootMaster_Position = masterMotor.getPosition().getValueAsDouble();
        inputs.ShootMaster_Velocity = masterMotor.getVelocity().getValueAsDouble();
        inputs.ShootMaster_AppliedVolts = masterMotor.getMotorVoltage().getValueAsDouble();
        inputs.ShootMaster_CurrentAmps = masterMotor.getStatorCurrent().getValueAsDouble();

        inputs.ShootSlave_Position = slaveMotor.getPosition().getValueAsDouble();
        inputs.ShootSlave_Velocity = slaveMotor.getVelocity().getValueAsDouble();
        inputs.ShootSlave_AppliedVolts = slaveMotor.getMotorVoltage().getValueAsDouble();
        inputs.ShootSlave_CurrentAmps = slaveMotor.getStatorCurrent().getValueAsDouble();

        inputs.AngleEncoder_RawPosition = absoluteEncoder.get();
    }

    public void applyEntryConfiguration(TalonFXConfiguration configuration) {
        entryMotor.getConfigurator().apply(configuration);
    }

    public void applyAngleConfiguration(TalonFXConfiguration configuration) {
        angleMotor.getConfigurator().apply(configuration);
    }

    public void applyMasterConfiguration(TalonFXConfiguration configuration) {
        masterMotor.getConfigurator().apply(configuration);
    }

    public void applySlaveConfiguration(TalonFXConfiguration configuration) {
        slaveMotor.getConfigurator().apply(configuration);
    }

    public void setEntryMotorPosition(double newValue) {
        entryMotor.setPosition(newValue);
    }

    public void setAngleMotorPosition(double newValue) {
        angleMotor.setPosition(newValue);
    }

    public void setMasterMotorPosition(double newValue) {
        masterMotor.setPosition(newValue);
    }

    public void setSlaveMotorPosition(double newValue) {
        slaveMotor.setPosition(newValue);
    }

    public void setEntryRequest(ControlRequest request) {
        entryMotor.setControl(request);
    }

    public void setAngleRequest(ControlRequest request) {
        angleMotor.setControl(request);
    }

    public void setMasterRequest(ControlRequest request) {
        masterMotor.setControl(request);
    }

    public void setSlaveRequest(ControlRequest request) {
        slaveMotor.setControl(request);
    }

    public void simulationPeriodic() {}

    @Override
    public void close() throws Exception {
        entryMotor.close();
        angleMotor.close();
        masterMotor.close();
        slaveMotor.close();
    }
}
