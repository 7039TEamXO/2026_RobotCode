package frc.robot.subsystems.IO.Real;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.subsystems.Intake.IntakeConstants;
import frc.robot.subsystems.IO.IntakeIO;

public class IntakeReal implements IntakeIO {
    private final TalonFX intakeMotor;
    private final TalonFX controlMotor;

    public IntakeReal() {        
        intakeMotor = new TalonFX(IntakeConstants.IntakeMotorID);
        controlMotor = new TalonFX(IntakeConstants.ControlMotorID);
    }

    public void updateInputs(IntakeIOInputs inputs) {
        inputs.Intake_Position = intakeMotor.getPosition().getValueAsDouble();
        inputs.Intake_Velocity = intakeMotor.getVelocity().getValueAsDouble();
        inputs.Intake_AppliedVolts = intakeMotor.getMotorVoltage().getValueAsDouble();
        inputs.Intake_CurrentAmps = intakeMotor.getStatorCurrent().getValueAsDouble();
    }

    public void applyIntakeConfiguration(TalonFXConfiguration configuration) {
        intakeMotor.getConfigurator().apply(configuration);
    }

    public void applyControlConfiguration(TalonFXConfiguration configuration) {
        controlMotor.getConfigurator().apply(configuration);
    }

    public void setControlPosition(double newValue) {
        controlMotor.setPosition(newValue);
    }

    public void setIntakeRequest(ControlRequest request) {
        intakeMotor.setControl(request);
    }

    public void setControlRequest(ControlRequest request) {
        controlMotor.setControl(request);
    }

    public void simulationPeriodic() {}

    @Override
    public void close() throws Exception {
        intakeMotor.close();
        controlMotor.close();
    }
}