package frc.robot.subsystems.IO;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;

public interface IntakeIO extends AutoCloseable {
    public static class IntakeIOInputs implements LoggableInputs {
        public double Intake_Position = 0.0;
        public double Intake_Velocity = 0.0;
        public double Intake_AppliedVolts = 0.0;
        public double Intake_CurrentAmps = 0.0;

        public double Control_Position = 0.0;
        public double Control_Velocity = 0.0;
        public double Control_AppliedVolts = 0.0;
        public double Control_CurrentAmps = 0.0;

        public void toLog(LogTable table) {
            table.put("Intake Position", Intake_Position);
            table.put("Intake Velocity", Intake_Velocity);
            table.put("Intake Applied Volts", Intake_AppliedVolts);
            table.put("Intake Current Amps", Intake_CurrentAmps);
            
            table.put("Control Position", Control_Position);
            table.put("Control Velocity", Control_Velocity);
            table.put("Control Applied Volts", Control_AppliedVolts);
            table.put("Control Current Amps", Control_CurrentAmps);
        }

        public void fromLog(LogTable table) {
            Intake_Position = table.get("Intake Position", 0.0);
            Intake_Velocity = table.get("Intake Velocity", 0.0);
            Intake_AppliedVolts = table.get("Intake Applied Volts", 0.0);
            Intake_CurrentAmps = table.get("Intake Current Amps", 0.0);
            
            Control_Position = table.get("Control Position", 0.0);
            Control_Velocity = table.get("Control Velocity", 0.0);
            Control_AppliedVolts = table.get("Control Applied Volts", 0.0);
            Control_CurrentAmps = table.get("Control Current Amps", 0.0);
        }
    }

    public void updateInputs(IntakeIOInputs inputs);
    public void applyIntakeConfiguration(TalonFXConfiguration configuration);
    public void applyControlConfiguration(TalonFXConfiguration configuration);
    public void setControlPosition(double newValue);
    public void setIntakeRequest(ControlRequest request);
    public void setControlRequest(ControlRequest request);

    public void simulationPeriodic();
}