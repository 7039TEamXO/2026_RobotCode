package frc.robot.subsystems.IO.Sim;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.Delivery.Shooter.ShooterConstants;
import frc.robot.subsystems.IO.ShooterIO;

public class ShooterSim implements ShooterIO {
    private final TalonFX entryMotor;
    private final TalonFX angleMotor;
    private final TalonFX masterMotor;
    private final TalonFX slaveMotor;

    private final TalonFXSimState entrySim;
    private final DCMotorSim entrySimModel;

    private final TalonFXSimState angleSim;
    private final DCMotorSim angleSimModel;

    private final TalonFXSimState masterSim;
    private final DCMotorSim masterSimModel;

    private final TalonFXSimState slaveSim;
    private final DCMotorSim slaveSimModel;

    private final double periodicDt;

    public ShooterSim(double _periodicDt) {
        periodicDt = _periodicDt;
        
        entryMotor = new TalonFX(ShooterConstants.ShooterEntryMotorID);
        entrySim = entryMotor.getSimState();
        entrySimModel = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60Foc(1), 0.001, ShooterConstants.SimulationGearRatio
            ),
            DCMotor.getKrakenX60Foc(1)
        );
    
        angleMotor = new TalonFX(ShooterConstants.ShooterAngleMotorID);
        angleSim = entryMotor.getSimState();
        angleSimModel = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60Foc(1), 0.001, ShooterConstants.SimulationGearRatio
            ),
            DCMotor.getKrakenX60Foc(1)
        );

        masterMotor = new TalonFX(ShooterConstants.ShooterMasterMotorID);
        masterSim = masterMotor.getSimState();
        masterSimModel = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60Foc(1), 0.001, ShooterConstants.SimulationGearRatio
            ),
            DCMotor.getKrakenX60Foc(1)
        );

        slaveMotor = new TalonFX(ShooterConstants.ShooterSlaveMotorID);
        slaveSim = masterMotor.getSimState();
        slaveSimModel = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60Foc(1), 0.001, ShooterConstants.SimulationGearRatio
            ),
            DCMotor.getKrakenX60Foc(1)
        );
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

    public void simulationPeriodic() {
        entrySim.setSupplyVoltage(RobotController.getBatteryVoltage());
        angleSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        masterSim.setSupplyVoltage(RobotController.getBatteryVoltage());
        slaveSim.setSupplyVoltage(RobotController.getBatteryVoltage());        
        
        double entryMotorVoltage = entrySim.getMotorVoltage();
        double angleMotorVoltage = angleSim.getMotorVoltage();
        double masterMotorVoltage = masterSim.getMotorVoltage();
        double slaveMotorVoltage = slaveSim.getMotorVoltage();

        entrySimModel.setInputVoltage(entryMotorVoltage);
        angleSimModel.setInputVoltage(angleMotorVoltage);
        masterSimModel.setInputVoltage(masterMotorVoltage);
        slaveSimModel.setInputVoltage(slaveMotorVoltage);
     
        entrySimModel.update(periodicDt);
        angleSimModel.update(periodicDt);
        masterSimModel.update(periodicDt);
        slaveSimModel.update(periodicDt);
    
        entrySim.setRawRotorPosition(entrySimModel.getAngularPosition().times(ShooterConstants.SimulationGearRatio));
        entrySim.setRotorVelocity(entrySimModel.getAngularVelocity().times(ShooterConstants.SimulationGearRatio));
        angleSim.setRawRotorPosition(angleSimModel.getAngularPosition().times(ShooterConstants.SimulationGearRatio));
        angleSim.setRotorVelocity(angleSimModel.getAngularVelocity().times(ShooterConstants.SimulationGearRatio));
        masterSim.setRawRotorPosition(masterSimModel.getAngularPosition().times(ShooterConstants.SimulationGearRatio));
        masterSim.setRotorVelocity(masterSimModel.getAngularVelocity().times(ShooterConstants.SimulationGearRatio));
        slaveSim.setRawRotorPosition(slaveSimModel.getAngularPosition().times(ShooterConstants.SimulationGearRatio));
        slaveSim.setRotorVelocity(slaveSimModel.getAngularVelocity().times(ShooterConstants.SimulationGearRatio));
    }

    @Override
    public void close() throws Exception {
        entryMotor.close();
        angleMotor.close();
        masterMotor.close();
        slaveMotor.close();
    }
}
