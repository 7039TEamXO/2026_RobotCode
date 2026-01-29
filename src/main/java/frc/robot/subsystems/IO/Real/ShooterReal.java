package frc.robot.subsystems.IO.Real;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.Delivery.Shooter.ShooterConstants;
import frc.robot.subsystems.IO.ShooterIO;

public class ShooterReal implements ShooterIO {
    private final TalonFX entryMotor;
    private final TalonFX angleMotor;
    private final TalonFX masterMotor;
    private final TalonFX slaveMotor;

    public ShooterReal() {
        entryMotor = new TalonFX(ShooterConstants.ShooterEntryMotorID);
        angleMotor = new TalonFX(ShooterConstants.ShooterAngleMotorID);
        masterMotor = new TalonFX(ShooterConstants.ShooterMasterMotorID);
        slaveMotor = new TalonFX(ShooterConstants.ShooterSlaveMotorID);
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
        inputs.ShootSlave_Position = slaveMotor.getVelocity().getValueAsDouble();
        inputs.ShootSlave_Position = slaveMotor.getMotorVoltage().getValueAsDouble();
        inputs.ShootSlave_Position = slaveMotor.getStatorCurrent().getValueAsDouble();
    }

    public void applyEntryTalonFXConfig(TalonFXConfiguration configuration) {
        entryMotor.getConfigurator().apply(configuration);
    }

    public void applyAngleTalonFXConfig(TalonFXConfiguration configuration) {
        angleMotor.getConfigurator().apply(configuration);
    }

    public void applyMasterTalonFXConfig(TalonFXConfiguration configuration) {
        masterMotor.getConfigurator().apply(configuration);
    }

    public void applySlaveTalonFXConfig(TalonFXConfiguration configuration) {
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

    public void setEntryMotionMagic(MotionMagicExpoVoltage request) {
        entryMotor.setControl(request);
    }

    public void setAngleMotionMagic(MotionMagicExpoVoltage request) {
        angleMotor.setControl(request);
    }

    public void setMasterMotionMagic(MotionMagicExpoVoltage request) {
        masterMotor.setControl(request);
    }

    public Follower createMasterFollower() {
        return new Follower(masterMotor.getDeviceID(), MotorAlignmentValue.Aligned);
    }

    public void setSlaveMotionMagic(Follower request) {
        slaveMotor.setControl(request);
    }

    public void setEntryMotorVoltage(Voltage voltage) {
        entryMotor.setVoltage(voltage.in(Volts));
    }

    public void setAngleMotorVoltage(Voltage voltage) {
        angleMotor.setVoltage(voltage.in(Volts));
    }

    public void setMasterMotorVoltage(Voltage voltage) {
        masterMotor.setVoltage(voltage.in(Volts));
    }

    public void setSlaveMotorVoltage(Voltage voltage) {
        slaveMotor.setVoltage(voltage.in(Volts));
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
