package frc.robot.subsystems.IO.Sim;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.Intake.IntakeConstants;
import frc.robot.subsystems.IO.IntakeIO;

public class IntakeSim implements IntakeIO {
    private final TalonFX intakeMotor;
    private final TalonFX controlMotor;

    private final TalonFXSimState intakeSim;
    private final DCMotorSim intakeSimModel;

    private final TalonFXSimState controlSim;
    private final DCMotorSim controlSimModel;

    private final double periodicDt;

    public IntakeSim(double _periodicDt) {       
        periodicDt = _periodicDt;

        intakeMotor = new TalonFX(IntakeConstants.IntakeMotorID);
        intakeSim = intakeMotor.getSimState();
        intakeSimModel = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60Foc(1), 0.001, IntakeConstants.SimulationMotorGearRatio
            ),
            DCMotor.getKrakenX60Foc(1)
        );
    
        controlMotor = new TalonFX(IntakeConstants.ControlMotorID);
        controlSim = controlMotor.getSimState();
        controlSimModel = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60Foc(1), 0.001, IntakeConstants.SimulationMotorGearRatio
            ),
            DCMotor.getKrakenX60Foc(1)
        );
    }

    public void updateInputs(IntakeIOInputs inputs) {
        inputs.Intake_Position = intakeMotor.getPosition().getValueAsDouble();
        inputs.Intake_Velocity = intakeMotor.getVelocity().getValueAsDouble();
        inputs.Intake_AppliedVolts = intakeMotor.getMotorVoltage().getValueAsDouble();
        inputs.Intake_CurrentAmps = intakeMotor.getStatorCurrent().getValueAsDouble();

        inputs.Control_Position = controlMotor.getPosition().getValueAsDouble();
        inputs.Control_Velocity = controlMotor.getVelocity().getValueAsDouble();
        inputs.Control_AppliedVolts = controlMotor.getMotorVoltage().getValueAsDouble();
        inputs.Control_CurrentAmps = controlMotor.getStatorCurrent().getValueAsDouble();
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

    public void simulationPeriodic() {
        intakeSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        controlSim.setSupplyVoltage(RobotController.getBatteryVoltage());
 
        double intakeMotorVoltage = intakeSim.getMotorVoltage();
        double controlMotorVoltage = controlSim.getMotorVoltage();

        intakeSimModel.setInputVoltage(intakeMotorVoltage);
        controlSimModel.setInputVoltage(controlMotorVoltage);

        intakeSimModel.update(periodicDt);
        controlSimModel.update(periodicDt);
    
        intakeSim.setRawRotorPosition(intakeSimModel.getAngularPosition().times(IntakeConstants.SimulationMotorGearRatio));
        intakeSim.setRotorVelocity(intakeSimModel.getAngularVelocity().times(IntakeConstants.SimulationMotorGearRatio));
        controlSim.setRawRotorPosition(controlSimModel.getAngularPosition().times(IntakeConstants.SimulationMotorGearRatio));
        controlSim.setRotorVelocity(controlSimModel.getAngularVelocity().times(IntakeConstants.SimulationMotorGearRatio));
    }

    @Override
    public void close() throws Exception {
        intakeMotor.close();
        controlMotor.close();
    }
}