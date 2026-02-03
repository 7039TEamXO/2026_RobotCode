package frc.robot.subsystems.IO;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;

public interface ConveyorIO extends AutoCloseable {
    public static class ConveyorIOInputs implements LoggableInputs {
        public double Position = 0.0;
        public double Velocity = 0.0;
        public double AppliedVolts = 0.0;
        public double CurrentAmps = 0.0;

        public void toLog(LogTable table) {
            table.put("Position", Position);
            table.put("Velocity", Velocity);
            table.put("Applied Volts", AppliedVolts);
            table.put("Current Amps", CurrentAmps);
        }

        public void fromLog(LogTable table) {
            Position = table.get("Position", 0.0);
            Velocity = table.get("Velocity", 0.0);
            AppliedVolts = table.get("Applied Volts", 0.0);
            CurrentAmps = table.get("Current Amps", 0.0);
        }
    }

    public void updateInputs(ConveyorIOInputs inputs);

    public void applyConfiguration(TalonFXConfiguration configuration);

    public void setPosition(double newValue);

    public void setRequest(ControlRequest request);

    public void simulationPeriodic();
}