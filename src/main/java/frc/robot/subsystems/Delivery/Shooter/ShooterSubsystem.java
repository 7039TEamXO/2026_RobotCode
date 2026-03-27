package frc.robot.subsystems.Delivery.Shooter;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.Constants;
import frc.robot.Dashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.TuningMode;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.IO.ShooterIO;
import frc.robot.subsystems.IO.ShooterIO.ShooterIOInputs;

public class ShooterSubsystem implements Subsystem {
    private final ShooterIO io;
    private final ShooterIOInputs inputs;

    private final InterpolatingDoubleTreeMap angleInterpolatingMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap shooterInterpolatingMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap timeInterpolatingMap = new InterpolatingDoubleTreeMap();

    public ShooterSubsystem(ShooterIO _io) {
        io = _io;
        inputs = new ShooterIOInputs();

        io.setAngleMotorPosition(0);

        setMotorConfigs();
        Logger.processInputs("Shooter", inputs);

        setupAngleInterpolatingMap();
        setupShooterInterpolatingMap();
        setupTimeInterpolatingMap();

        Shoot_SysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.per(Second).of(ShooterConstants.Shoot_SysIdQuasistatic), 
                Volts.of(ShooterConstants.Shoot_SysIdDynamic), null, state -> {
                    SignalLogger.writeString("state", state.toString());
            }),
            new SysIdRoutine.Mechanism(v -> {
                io.setMasterRequest(new VoltageOut(v));

                io.updateInputs(inputs);
                Logger.processInputs("Shooter", inputs);
            }, null, this));

        Angle_SysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.per(Second).of(ShooterConstants.Angle_SysIdQuasistatic), 
                Volts.of(ShooterConstants.Angle_SysIdDynamic), null, state -> {
                    SignalLogger.writeString("state", state.toString());
            }),
            new SysIdRoutine.Mechanism(v -> {
                io.setAngleRequest(new VoltageOut(v));

                io.updateInputs(inputs);
                Logger.processInputs("Shooter", inputs);
            }, null, this));

        Entry_SysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.per(Second).of(ShooterConstants.Entry_SysIdQuasistatic), 
                Volts.of(ShooterConstants.Entry_SysIdDynamic), null, state -> {
                    SignalLogger.writeString("state", state.toString());
            }),
            new SysIdRoutine.Mechanism(v -> {
                io.setEntryRequest(new VoltageOut(v));

                io.updateInputs(inputs);
                Logger.processInputs("Shooter", inputs);
            }, null, this));
    }

    public void operate(ShooterState state) {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);

        if(Constants.GetTuningMode() == TuningMode.ACTIVE) setMotorConfigs();

        double Entry_Velocity = ShooterConstants.ENTRY_VELOCITY_STOP;
        double Angle_Position = calculateShooterAngle();
        double Shoot_Velocity = ShooterTuning.SHOOT_VELOCITY_IDLE_get();
        
        switch (state) {
            case INTAKE_HELP:
                Entry_Velocity = ShooterTuning.ENTRY_VELOCITY_ROLL_get();
                Shoot_Velocity = ShooterTuning.SHOOT_VELOCITY_HELP_get();
                Angle_Position = ShooterTuning.ANGLE_INTAKE_HELP_POS_get();
                break;
            case DEPLETE:
                Entry_Velocity = ShooterTuning.ENTRY_VELOCITY_REVERSE_get();
                Angle_Position = ShooterTuning.ANGLE_INTAKE_HELP_POS_get();
                break;
            case SPEED_UP:
                Shoot_Velocity = calculateShooterVelocity();
                break;
            case SHOOT:
                Entry_Velocity = ShooterTuning.ENTRY_VELOCITY_ROLL_get();
                Shoot_Velocity = calculateShooterVelocity();
                break;
            default:
            case IDLE:
                break;
        }

        io.setEntryRequest(new VelocityVoltage(Entry_Velocity));
        io.setAngleRequest(new MotionMagicVoltage(Angle_Position));
        io.setMasterRequest(new VelocityVoltage(Shoot_Velocity));
    }

    public ShooterIOInputs getInputs() {
        return inputs;
    }
    
    private void setupAngleInterpolatingMap() {
        final double angleCalibrationOffset = 0.7;
        angleInterpolatingMap.put(0.0, 2.0 + angleCalibrationOffset);
        angleInterpolatingMap.put(1.5, 2.5 + angleCalibrationOffset);
        angleInterpolatingMap.put(2.0, 3.0 + angleCalibrationOffset);
        angleInterpolatingMap.put(2.5, 3.5 + angleCalibrationOffset);
        angleInterpolatingMap.put(3.0, 4.0 + angleCalibrationOffset);
        angleInterpolatingMap.put(4.0, 4.5 + angleCalibrationOffset);
        angleInterpolatingMap.put(4.5, 5.0 + angleCalibrationOffset);
        angleInterpolatingMap.put(5.0, 5.0 + angleCalibrationOffset);
        angleInterpolatingMap.put(5.5, 5.5 + angleCalibrationOffset);

        // Consistency
        angleInterpolatingMap.put(10.0, ShooterConstants.ANGLE_SHOOT_MAX);    }

    private void setupShooterInterpolatingMap() {
        final double shooterCalibrationOffset = -1.0;
        shooterInterpolatingMap.put(0.0, 42.0 + shooterCalibrationOffset);
        shooterInterpolatingMap.put(1.5, 45.0 + shooterCalibrationOffset);
        shooterInterpolatingMap.put(2.0, 47.0 + shooterCalibrationOffset);
        shooterInterpolatingMap.put(2.5, 49.0 + shooterCalibrationOffset);
        shooterInterpolatingMap.put(3.0, 50.0 + shooterCalibrationOffset);
        shooterInterpolatingMap.put(4.0, 55.0 + shooterCalibrationOffset);
        shooterInterpolatingMap.put(4.5, 56.0 + shooterCalibrationOffset);
        shooterInterpolatingMap.put(5.0, 60.0 + shooterCalibrationOffset);
        shooterInterpolatingMap.put(5.5, 62.0 + shooterCalibrationOffset);
        
        // Consistency
        shooterInterpolatingMap.put(10.0, ShooterConstants.RPS_SHOOT_MAX);
    }

    private void setupTimeInterpolatingMap() {
        timeInterpolatingMap.put(0.0, 1.0);
        timeInterpolatingMap.put(1.5, 1.5);
        timeInterpolatingMap.put(2.0, 1.5);
        timeInterpolatingMap.put(2.5, 1.5);
        timeInterpolatingMap.put(3.0, 2.0);
        timeInterpolatingMap.put(4.0, 2.0);
        timeInterpolatingMap.put(4.5, 2.0);
        timeInterpolatingMap.put(5.0, 2.5);
        timeInterpolatingMap.put(5.5, 2.5);

        // Consistency
        timeInterpolatingMap.put(10.0, ShooterConstants.TIME_SHOOT_MAX);
    }

    public double calculateShooterAngle() {
        if(Dashboard.getAcceptAngleChanges()) return Dashboard.getAngleValue();
        if(SubsystemManager.getDrivebase().isInNeutralZone()) return ShooterTuning.ANGLE_NEUTRAL_ZONE_POS_get();
        double distanceFromHub = SubsystemManager.getDrivebase().calculateDistanceToHub(true);
        return angleInterpolatingMap.get(distanceFromHub);
    }

    public double calculateShooterVelocity() {
        if(Dashboard.getAcceptVelocityChanges()) return Dashboard.getVelocityValue();
        if(SubsystemManager.getDrivebase().isInNeutralZone()) return ShooterTuning.SHOOT_VELOCITY_NEUTRAL_ZONE_get();
        double distanceFromHub = SubsystemManager.getDrivebase().calculateDistanceToHub(true);
        return shooterInterpolatingMap.get(distanceFromHub);
    }

    public double calculateFlightTime(double distance) {
        // No Dashboard tuning required
        return timeInterpolatingMap.get(distance);
    }

    public boolean isStuck() {
        return inputs.ShootMaster_Velocity < ShooterTuning.SHOOT_VELOCITY_MIN_get() || inputs.EntryMotor_Velocity < ShooterTuning.ENTRY_VELOCITY_MIN_get();
    }

    private void setEntryMotorConfigs() {
        TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
        Slot0Configs slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = ShooterTuning.Entry_kS_get();
        slot0Configs.kV = ShooterTuning.Entry_kV_get();
        slot0Configs.kA = ShooterTuning.Entry_kA_get();
        slot0Configs.kP = ShooterTuning.Entry_kP_get();
        slot0Configs.kI = ShooterTuning.Entry_kI_get();
        slot0Configs.kD = ShooterTuning.Entry_kD_get();

        talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        talonFXConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // Inverted
        talonFXConfigs.CurrentLimits.StatorCurrentLimit = ShooterConstants.Entry_StatorCurrentLimit;
        talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        talonFXConfigs.CurrentLimits.SupplyCurrentLimit = ShooterConstants.Entry_SupplyCurrentLimit;
        talonFXConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

        io.applyEntryConfiguration(talonFXConfigs);
    }

    private void setAngleMotorConfigs() {
        TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
        Slot0Configs slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = ShooterTuning.Angle_kS_get();
        slot0Configs.kV = ShooterTuning.Angle_kV_get();
        slot0Configs.kA = ShooterTuning.Angle_kA_get();
        slot0Configs.kP = ShooterTuning.Angle_kP_get();
        slot0Configs.kI = ShooterTuning.Angle_kI_get();
        slot0Configs.kD = ShooterTuning.Angle_kD_get();

        MotionMagicConfigs motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = ShooterTuning.Angle_CruiseVelocity_get();
        motionMagicConfigs.MotionMagicAcceleration =  ShooterTuning.Angle_Acceleration_get();
        motionMagicConfigs.MotionMagicJerk = ShooterTuning.Angle_Jerk_get();

        talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        talonFXConfigs.CurrentLimits.StatorCurrentLimit = ShooterConstants.Angle_StatorCurrentLimit;
        talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        talonFXConfigs.CurrentLimits.SupplyCurrentLimit = ShooterConstants.Angle_SupplyCurrentLimit;
        talonFXConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

        io.applyAngleConfiguration(talonFXConfigs);
    }

    private void setShootMotorsConfigs() {
        io.setSlaveRequest(new Follower(ShooterConstants.ShooterMasterMotorID, MotorAlignmentValue.Opposed));

        TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
        Slot0Configs slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = ShooterTuning.Shoot_kS_get();
        slot0Configs.kV = ShooterTuning.Shoot_kV_get();
        slot0Configs.kA = ShooterTuning.Shoot_kA_get();
        slot0Configs.kP = ShooterTuning.Shoot_kP_get();
        slot0Configs.kI = ShooterTuning.Shoot_kI_get();
        slot0Configs.kD = ShooterTuning.Shoot_kD_get();

        talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        talonFXConfigs.CurrentLimits.StatorCurrentLimit = ShooterConstants.Shoot_StatorCurrentLimit;
        talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        talonFXConfigs.CurrentLimits.SupplyCurrentLimit = ShooterConstants.Shoot_SupplyCurrentLimit;
        talonFXConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

        io.applyMasterConfiguration(talonFXConfigs);
        io.applySlaveConfiguration(talonFXConfigs);
    }

    private void setMotorConfigs() {
        setEntryMotorConfigs();
        setAngleMotorConfigs();
        setShootMotorsConfigs();
    }

    public void simulationPeriodic() {
        io.simulationPeriodic();
    }

    // SysId routines //

    private SysIdRoutine Shoot_SysIdRoutine;
    private SysIdRoutine Angle_SysIdRoutine;
    private SysIdRoutine Entry_SysIdRoutine;

    public Command Shoot_SysIdQuasistatic(SysIdRoutine.Direction direction) {
        return Shoot_SysIdRoutine.quasistatic(direction);
    }

    public Command Shoot_SysIdDynamic(SysIdRoutine.Direction direction) {
        return Shoot_SysIdRoutine.dynamic(direction);
    }

    public Command Angle_SysIdQuasistatic(SysIdRoutine.Direction direction) {
        return Angle_SysIdRoutine.quasistatic(direction);
    }

    public Command Angle_SysIdDynamic(SysIdRoutine.Direction direction) {
        return Angle_SysIdRoutine.dynamic(direction);
    }

    public Command Entry_SysIdQuasistatic(SysIdRoutine.Direction direction) {
        return Entry_SysIdRoutine.quasistatic(direction);
    }

    public Command Entry_SysIdDynamic(SysIdRoutine.Direction direction) {
        return Entry_SysIdRoutine.dynamic(direction);
    }
}
