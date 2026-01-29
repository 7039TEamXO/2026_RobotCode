package frc.robot.subsystems.IO.Sim;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.IO.IntakeIO;
import frc.robot.subsystems.Intake.IntakeConstants;

public class IntakeSim implements IntakeIO {
    private final TalonFX master;
    private final TalonFXSimState masterSim;
    private final DCMotorSim motorSimModel;

    private final double periodicDt;

    private static final double GearRatio = 10.0;

    public IntakeSim(double _periodicDt) {
        periodicDt = _periodicDt;

        master = new TalonFX(IntakeConstants.IntakeMotorID);
        masterSim = master.getSimState();
        motorSimModel = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60Foc(1), 0.001, GearRatio
            ),
            DCMotor.getKrakenX60Foc(1)
        );
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.position = master.getPosition().getValueAsDouble();
        inputs.velocity = master.getVelocity().getValueAsDouble();
        inputs.appliedVolts = master.getMotorVoltage().getValueAsDouble();
        inputs.currentAmps = master.getStatorCurrent().getValueAsDouble();
    }

    @Override
    public void applyTalonFXConfig(TalonFXConfiguration configuration) {
        master.getConfigurator().apply(configuration);
    }

    @Override
    public void setPosition(double newValue) {
        master.setPosition(newValue);
    }

    @Override
    public void setMotionMagic(DutyCycleOut request) {
        master.setControl(request);
    }

    @Override
    public void setVoltage(Voltage voltage) {
        master.setVoltage(voltage.in(Volts));
    }

    @Override
    public void simulationPeriodic() {
        masterSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        
        var motorVoltage = masterSim.getMotorVoltage();

        motorSimModel.setInputVoltage(motorVoltage);
     
        motorSimModel.update(periodicDt);
    
        masterSim.setRawRotorPosition(motorSimModel.getAngularPosition().times(GearRatio));
        masterSim.setRotorVelocity(motorSimModel.getAngularVelocity().times(GearRatio));
    }

    @Override
    public void close() throws Exception {
        master.close();
    }
}