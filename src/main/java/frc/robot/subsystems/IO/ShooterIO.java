package frc.robot.subsystems.IO;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;

public interface ShooterIO extends AutoCloseable {
    public static class ShooterIOInputs implements LoggableInputs {
        public double EntryMotor_Position = 0.0;
        public double EntryMotor_Velocity = 0.0;
        public double EntryMotor_AppliedVolts = 0.0;
        public double EntryMotor_CurrentAmps = 0.0;

        public double AngleMotor_Position = 0.0;
        public double AngleMotor_Velocity = 0.0;
        public double AngleMotor_AppliedVolts = 0.0;
        public double AngleMotor_CurrentAmps = 0.0;
        
        public double ShootMaster_Position = 0.0;
        public double ShootMaster_Velocity = 0.0;
        public double ShootMaster_AppliedVolts = 0.0;
        public double ShootMaster_CurrentAmps = 0.0;

        public double ShootSlave_Position = 0.0;
        public double ShootSlave_Velocity = 0.0;
        public double ShootSlave_AppliedVolts = 0.0;
        public double ShootSlave_CurrentAmps = 0.0;

        public double AngleEncoder_RawPosition = 0.0;

        public void toLog(LogTable table) {
            table.put("Entry Motor Position", EntryMotor_Position);
            table.put("Entry Motor Velocity", EntryMotor_Velocity);
            table.put("Entry Motor Applied Volts", EntryMotor_AppliedVolts);
            table.put("Entry Motor Current Amps", EntryMotor_CurrentAmps);

            table.put("Angle Motor Position", AngleMotor_Position);
            table.put("Angle Motor Velocity", AngleMotor_Velocity);
            table.put("Angle Motor Applied Volts", AngleMotor_AppliedVolts);
            table.put("Angle Motor Current Amps", AngleMotor_CurrentAmps);

            table.put("Shoot Master Position", ShootMaster_Position);
            table.put("Shoot Master Velocity", ShootMaster_Velocity);
            table.put("Shoot Master Applied Volts", ShootMaster_AppliedVolts);
            table.put("Shoot Master Current Amps", ShootMaster_CurrentAmps);

            table.put("Shoot Slave Position", ShootSlave_Position);
            table.put("Shoot Slave Velocity", ShootSlave_Velocity);
            table.put("Shoot Slave Applied Volts", ShootSlave_AppliedVolts);
            table.put("Shoot Slave Current Amps", ShootSlave_CurrentAmps);

            table.put("Angle Encoder Raw Position", AngleEncoder_RawPosition);
        }

        public void fromLog(LogTable table) {
            EntryMotor_Position = table.get("Entry Motor Position", 0.0);
            EntryMotor_Velocity = table.get("Entry Motor Velocity", 0.0);
            EntryMotor_AppliedVolts = table.get("Entry Motor Applied Volts", 0.0);
            EntryMotor_CurrentAmps = table.get("Entry Motor Current Amps", 0.0);

            AngleMotor_Position = table.get("Angle Motor Position", 0.0);
            AngleMotor_Velocity = table.get("Angle Motor Velocity", 0.0);
            AngleMotor_AppliedVolts = table.get("Angle Motor Applied Volts", 0.0);
            AngleMotor_CurrentAmps = table.get("Angle Motor Current Amps", 0.0);

            ShootMaster_Position = table.get("Shoot Master Position", 0.0);
            ShootMaster_Velocity = table.get("Shoot Master Velocity", 0.0);
            ShootMaster_AppliedVolts = table.get("Shoot Master Applied Volts", 0.0);
            ShootMaster_CurrentAmps = table.get("Shoot Master Current Amps", 0.0);

            ShootSlave_Position = table.get("Shoot Slave Position", 0.0);
            ShootSlave_Velocity = table.get("Shoot Slave Velocity", 0.0);
            ShootSlave_AppliedVolts = table.get("Shoot Slave Applied Volts", 0.0);
            ShootSlave_CurrentAmps = table.get("Shoot Slave Current Amps", 0.0);

            AngleEncoder_RawPosition = table.get("Angle Encoder Raw Position", 0.0);
        }
    }

    public void updateInputs(ShooterIOInputs inputs);

    public void applyEntryConfiguration(TalonFXConfiguration configuration);
    public void applyAngleConfiguration(TalonFXConfiguration configuration);
    public void applyMasterConfiguration(TalonFXConfiguration configuration);
    public void applySlaveConfiguration(TalonFXConfiguration configuration);

    public void setAngleMotorPosition(double newValue);

    public void setEntryRequest(ControlRequest request);
    public void setAngleRequest(ControlRequest request);
    public void setMasterRequest(ControlRequest request);
    public void setSlaveFollower(Follower request);

    public void simulationPeriodic();
}
