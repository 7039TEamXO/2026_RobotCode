package frc.robot.subsystems.IO.Sim;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.Delivery.Conveyor.ConveyorConstants;
import frc.robot.subsystems.IO.ConveyorIO;

public class ConveyorSim implements ConveyorIO {
    private final TalonFX motor;

    private final TalonFXSimState motorSim;
    private final DCMotorSim motorSimModel;

    private final double periodicDt;

    public ConveyorSim(double _periodicDt) {
        periodicDt = _periodicDt;

        motor = new TalonFX(ConveyorConstants.ConveyorMotorID);

        motorSim = motor.getSimState();
        motorSimModel = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60Foc(1), 0.001, ConveyorConstants.SimulationMotorGearRatio
            ),
            DCMotor.getKrakenX60Foc(1)
        );
    }

    public void updateInputs(ConveyorIOInputs inputs) {
        inputs.Position = motor.getPosition().getValueAsDouble();
        inputs.Velocity = motor.getVelocity().getValueAsDouble();
        inputs.AppliedVolts = motor.getMotorVoltage().getValueAsDouble();
        inputs.CurrentAmps = motor.getStatorCurrent().getValueAsDouble();
    }

    public void applyConfiguration(TalonFXConfiguration configuration) {
        motor.getConfigurator().apply(configuration);
    }

    public void setPosition(double newValue) {
        motor.setPosition(newValue);
    }

    public void setRequest(ControlRequest request) {
        motor.setControl(request);
    }

    public void simulationPeriodic() {
        motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        double motorVoltage = motorSim.getMotorVoltage();

        motorSimModel.setInputVoltage(motorVoltage);

        motorSimModel.update(periodicDt);
    
        motorSim.setRawRotorPosition(motorSimModel.getAngularPosition().times(ConveyorConstants.SimulationMotorGearRatio));
        motorSim.setRotorVelocity(motorSimModel.getAngularVelocity().times(ConveyorConstants.SimulationMotorGearRatio));
    }

    @Override
    public void close() throws Exception {
        motor.close();
    }
}