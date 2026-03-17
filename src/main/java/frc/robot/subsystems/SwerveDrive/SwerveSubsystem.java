// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SwerveDrive;

import com.ctre.phoenix6.SignalLogger;
import com.therekrab.autopilot.APTarget;
import com.therekrab.autopilot.Autopilot.APResult;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Limelight;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.LimelightSimMode;
import frc.robot.Constants.TuningMode;
import frc.robot.commands.CollectAuto;
import frc.robot.commands.ShootAuto;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.Delivery.Shooter.ShooterState;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import java.io.File;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {
  private SwerveDrive swerveDrive;
  // private SwerveDrive swerveDriveOdometry; // Tuning

  public SwerveSubsystem(File directory) {
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    try {
      SimulatedArena.overrideInstance(new Arena2026Rebuilt(false));

      swerveDrive = new SwerveParser(directory).createSwerveDrive(SwerveDriveConstants.MAX_SPEED);
      // swerveDriveOdometry = new SwerveParser(directory).createSwerveDrive(SwerveDriveConstants.MAX_SPEED);
    } catch (Exception e) {
      throw new RuntimeException(e);
    }

    swerveDrive.setMotorIdleMode(true);
    swerveDrive.setHeadingCorrection(false); // Heading correction should only be used while controlling the robot via angle.
    swerveDrive.setCosineCompensator(false); // !SwerveDriveTelemetry.isSimulation); // Disables cosine compensation for
                                             // simulations since it causes discrepancies not seen in real life.
    swerveDrive.setAngularVelocityCompensation(true, true, 0.1);
    swerveDrive.setModuleEncoderAutoSynchronize(false, 1);

    // SysId
    driveSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
        Volts.per(Second).of(SwerveDriveConstants.DriveSysIdQuasistatic), 
        Volts.of(SwerveDriveConstants.DriveSysIdDynamic), null, state -> {
          SignalLogger.writeString("state", state.toString());
      }),
      new SysIdRoutine.Mechanism(v -> {
        for(int i = 0; i < 4; i++) {
          swerveDrive.getModules()[i].getDriveMotor().setVoltage(v.magnitude());
          swerveDrive.getModules()[i].setAngle(0);
        }
      }, null, this));

    turnSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(
        Volts.per(Second).of(SwerveDriveConstants.TurnSysIdQuasistatic), 
        Volts.of(SwerveDriveConstants.TurnSysIdDynamic), null, state -> {
          SignalLogger.writeString("state", state.toString());
      }),
      new SysIdRoutine.Mechanism(v -> {
        for(int i = 0; i < 4; i++) {
          swerveDrive.getModules()[i].getAngleMotor().setVoltage(v.magnitude());
        }
      }, null, this));
  }

  public SwerveSubsystem(SwerveDriveConfiguration driveCfg, SwerveControllerConfiguration controllerCfg) {
    swerveDrive = new SwerveDrive(driveCfg, controllerCfg, SwerveDriveConstants.MAX_SPEED,
        new Pose2d(new Translation2d(Meter.of(0), Meter.of(0)), Rotation2d.fromDegrees(0)));
  }

  @Override
  public void periodic() {
    if(Constants.GetTuningMode() == TuningMode.ACTIVE) {
      driveXPID.setPID(SwerveDriveTuning.KP_get(), 0, SwerveDriveTuning.KD_get());
      driveYPID.setPID(SwerveDriveTuning.KP_get(), 0, SwerveDriveTuning.KD_get());
      rotationPID.setPID(SwerveDriveTuning.KP_ANGULAR_get(), 0, SwerveDriveTuning.KD_ANGULAR_get());
      slickRotationPID.setPID(SwerveDriveTuning.KP_ANGULAR_SLICK_get(), 0, SwerveDriveTuning.KD_ANGULAR_SLICK_get());
    }

    // if(isAuto) filter by isRobotVBelowOne too
    Object[] tuple = Limelight.update();
    if (tuple != null && (double)tuple[3] > Constants.Limelight.TAForUpdate) {
      swerveDrive.addVisionMeasurement((Pose2d)tuple[0], (double)tuple[1]);
    }

    swerveDrive.updateOdometry();
    // swerveDriveOdometry.updateOdometry(); // Tuning

    Logger.recordOutput("Odometry/Pose", getPose());
    if(Constants.CurrentMode != frc.robot.Constants.Mode.REAL) Logger.recordOutput("Odometry/RealPose", getSimulationPose());
    if(!SubsystemManager.isInAutoDrive()) {
      Logger.recordOutput("Odometry/RequestedPose", new Pose2d(Double.MAX_VALUE, Double.MAX_VALUE, new Rotation2d()));
    }
    // Logger.recordOutput("Odometry/BasicPose", swerveDriveOdometry.getPose()); // Tuning
  }

  @Override
  public void simulationPeriodic() {}

  /* PROXIMITY VERIFIERS */

  public boolean isCloseEnoughToAngle(Rotation2d rotation, double cap) {
    double angleError = rotation.getDegrees() - getPose().getRotation().getDegrees();

    if(angleError > 180) angleError -= 360;
    if(angleError < -180) angleError += 360; // Degrees

    return Math.abs(angleError) < cap;
  }

  public boolean isCloseEnoughToPose(Pose2d pose, double distance, double angle) {
    return Math.abs(getPose().getX() - pose.getX()) < distance &&
      Math.abs(getPose().getY() - pose.getY()) < distance &&
      isCloseEnoughToAngle(pose.getRotation(), angle);
  }

  public boolean isCloseEnoughToPose(Pose2d pose, double distance) {
    return isCloseEnoughToPose(pose, distance, SwerveDriveTuning.CLOSE_ANGLE_ERROR_CAP_get());
  }

  public boolean isCloseEnoughToPose(Pose2d pose) {
    return isCloseEnoughToPose(pose, SwerveDriveTuning.CLOSE_DISTANCE_ERROR_CAP_get(), SwerveDriveTuning.CLOSE_ANGLE_ERROR_CAP_get());
  }

  public boolean isVxVelocityBelowThreshold() {
    return Math.abs(swerveDrive.getRobotVelocity().vxMetersPerSecond) < SwerveDriveTuning.VX_VELOCITY_THRESHOLD_get();
  }

  public boolean isVyVelocityBelowThreshold() {
    return Math.abs(swerveDrive.getRobotVelocity().vyMetersPerSecond) < SwerveDriveTuning.VY_VELOCITY_THRESHOLD_get();
  }

  public boolean isOmegaVelocityBelowThreshold() {
    return Math.abs(swerveDrive.getRobotVelocity().omegaRadiansPerSecond) < SwerveDriveTuning.OMEGA_VELOCITY_THRESHOLD_get();
  }

  public boolean isRobotVelocityBelowThreshold() {
    return isVxVelocityBelowThreshold() && isVyVelocityBelowThreshold() && isOmegaVelocityBelowThreshold();
  }

  /* DRIVE TO POSE COMMANDS (+ a lot of overloading) */

  private PIDController driveXPID = new PIDController(SwerveDriveConstants.KP, 0, SwerveDriveConstants.KD);
  private PIDController driveYPID = new PIDController(SwerveDriveConstants.KP, 0, SwerveDriveConstants.KD);
  private PIDController rotationPID = new PIDController(SwerveDriveConstants.KP_ANGULAR, 0, SwerveDriveConstants.KD_ANGULAR);
  private PIDController slickRotationPID = new PIDController(SwerveDriveConstants.KP_ANGULAR_SLICK, 0, SwerveDriveConstants.KD_ANGULAR_SLICK);

  public Command driveToPose(Pose2d pose, Boolean autoFinish) {
    return run(() -> {
      Logger.recordOutput("Odometry/RequestedPose", pose);

      // Drive compute
      ChassisSpeeds robotRelativeSpeeds = getRobotVelocity();
      APTarget target = new APTarget(pose);
      APResult output = SwerveDriveConstants.kAutopilot.calculate(getPose(), robotRelativeSpeeds, target);

      // Rotation compute
      PIDController R = rotationPID;

      // Ensuring that the error is in the -180 ... +180 degree range
      double robotAngle = getPose().getRotation().getRadians();

      R.setSetpoint(output.targetAngle().getRadians());
      double angleError = robotAngle - R.getSetpoint();

      if(angleError > Math.PI) robotAngle -= 2 * Math.PI;
      if(angleError < -Math.PI) robotAngle += 2 * Math.PI;

      Translation2d fieldRelative = new Translation2d(output.vx().in(MetersPerSecond), output.vy().in(MetersPerSecond));

      double omegaRadiansPerSecond = R.calculate(robotAngle);

      if(isCloseEnoughToPose(pose)) {
        PIDController X = driveXPID;
        PIDController Y = driveYPID;

        X.setSetpoint(pose.getX());
        Y.setSetpoint(pose.getY());

        fieldRelative = new Translation2d(X.calculate(getPose().getX()), Y.calculate(getPose().getY()));
      }

      swerveDrive.drive(fieldRelative, -omegaRadiansPerSecond, true, false);
    }).until(() -> {
      if(autoFinish) return isCloseEnoughToPose(pose);
      else return false;
    });
  }

  public Command driveToPose(APTarget target, Boolean autoFinish) {
    return run(() -> {
      Logger.recordOutput("Odometry/RequestedPose", target.getReference());

      // Drive compute
      ChassisSpeeds robotRelativeSpeeds = getRobotVelocity();
      APResult output = SwerveDriveConstants.kAutopilot.calculate(getPose(), robotRelativeSpeeds, target);

      // Rotation compute
      PIDController R = rotationPID;

      // Ensuring that the error is in the -180 ... +180 degree range
      double robotAngle = getPose().getRotation().getRadians();

      R.setSetpoint(output.targetAngle().getRadians());
      double angleError = robotAngle - R.getSetpoint();

      if(angleError > Math.PI) robotAngle -= 2 * Math.PI;
      if(angleError < -Math.PI) robotAngle += 2 * Math.PI;

      Translation2d fieldRelative = new Translation2d(output.vx().in(MetersPerSecond), output.vy().in(MetersPerSecond));
      double omegaRadiansPerSecond = R.calculate(robotAngle);

      if(isCloseEnoughToPose(target.getReference())) {
        PIDController X = driveXPID;
        PIDController Y = driveYPID;

        X.setSetpoint(target.getReference().getX());
        Y.setSetpoint(target.getReference().getY());

        fieldRelative = new Translation2d(X.calculate(getPose().getX()), Y.calculate(getPose().getY()));
      }

      swerveDrive.drive(fieldRelative, -omegaRadiansPerSecond, true, false);
    }).until(() -> {
      if(autoFinish) return isCloseEnoughToPose(target.getReference());
      else return false;
    });
  }

  public Command driveToPose(APTarget target, double distance, double angle) {
    return run(() -> {
      Logger.recordOutput("Odometry/RequestedPose", target.getReference());

      // Drive compute
      ChassisSpeeds robotRelativeSpeeds = getRobotVelocity();
      APResult output = SwerveDriveConstants.kAutopilot.calculate(getPose(), robotRelativeSpeeds, target);

      // Rotation compute
      PIDController R = rotationPID;

      // Ensuring that the error is in the -180 ... +180 degree range
      double robotAngle = getPose().getRotation().getRadians();

      R.setSetpoint(output.targetAngle().getRadians());
      double angleError = robotAngle - R.getSetpoint();

      if(angleError > Math.PI) robotAngle -= 2 * Math.PI;
      if(angleError < -Math.PI) robotAngle += 2 * Math.PI;

      Translation2d fieldRelative = new Translation2d(output.vx().in(MetersPerSecond), output.vy().in(MetersPerSecond));
      double omegaRadiansPerSecond = R.calculate(robotAngle);

      swerveDrive.drive(fieldRelative, -omegaRadiansPerSecond, true, false);
    }).until(() -> {
      return isCloseEnoughToPose(target.getReference(), distance, angle);
    });
  }

  private Integer waypointNumber = 0;

  public Command waypointToPose(APTarget[] targets, double switchDistance, Boolean autoFinish) {
    return run(() -> {
      Logger.recordOutput("Odometry/RequestedPose", targets[waypointNumber].getReference());

      // Drive compute
      ChassisSpeeds robotRelativeSpeeds = getRobotVelocity();
      APTarget target = targets[waypointNumber];
      APResult output = SwerveDriveConstants.kAutopilot.calculate(getPose(), robotRelativeSpeeds, target);

      // Rotation compute
      PIDController R = rotationPID;

      // Ensuring that the error is in the -180 ... +180 degree range
      double robotAngle = getPose().getRotation().getRadians();

      R.setSetpoint(output.targetAngle().getRadians());
      double angleError = robotAngle - R.getSetpoint();

      if(angleError > Math.PI) robotAngle -= 2 * Math.PI;
      if(angleError < -Math.PI) robotAngle += 2 * Math.PI;

      Translation2d fieldRelative = new Translation2d(output.vx().in(MetersPerSecond), output.vy().in(MetersPerSecond));
      double omegaRadiansPerSecond = R.calculate(robotAngle);

      // Hopping onto the next waypoint
      if(isCloseEnoughToPose(targets[waypointNumber].getReference(), switchDistance, 1e5)) {
        if(waypointNumber < targets.length - 1) waypointNumber++; 
      }

      // Switch to PID at the endpoint
      if(isCloseEnoughToPose(targets[targets.length - 1].getReference()) &&
          waypointNumber == targets.length - 1) {
        PIDController X = driveXPID;
        PIDController Y = driveYPID;

        X.setSetpoint(targets[targets.length - 1].getReference().getX());
        Y.setSetpoint(targets[targets.length - 1].getReference().getY());

        fieldRelative = new Translation2d(X.calculate(getPose().getX()), Y.calculate(getPose().getY()));
      }

      swerveDrive.drive(fieldRelative, -omegaRadiansPerSecond, true, false);
    }).until(() -> {
      if(autoFinish) {
        Boolean condition = isCloseEnoughToPose(targets[targets.length - 1].getReference()) &&
          waypointNumber == targets.length - 1;
        if(condition) SubsystemManager.getDrivebase().resetWaypointToPose();
        return condition;
      } else return false;
    });
  }

  public Command waypointToPose(APTarget[] targets, double switchDistance, double endDistance, double endAngle) {
    return run(() -> {
      Logger.recordOutput("Odometry/RequestedPose", targets[waypointNumber].getReference());

      // Drive compute
      ChassisSpeeds robotRelativeSpeeds = getRobotVelocity();
      APTarget target = targets[waypointNumber];
      APResult output = SwerveDriveConstants.kAutopilot.calculate(getPose(), robotRelativeSpeeds, target);

      // Rotation compute
      PIDController R = rotationPID;

      // Ensuring that the error is in the -180 ... +180 degree range
      double robotAngle = getPose().getRotation().getRadians();

      R.setSetpoint(output.targetAngle().getRadians());
      double angleError = robotAngle - R.getSetpoint();

      if(angleError > Math.PI) robotAngle -= 2 * Math.PI;
      if(angleError < -Math.PI) robotAngle += 2 * Math.PI;

      Translation2d fieldRelative = new Translation2d(output.vx().in(MetersPerSecond), output.vy().in(MetersPerSecond));
      double omegaRadiansPerSecond = R.calculate(robotAngle);

      // Hopping onto the next waypoint
      if(isCloseEnoughToPose(targets[waypointNumber].getReference(), switchDistance, 1e5)) {
        if(waypointNumber < targets.length - 1) waypointNumber++; 
      }

      swerveDrive.drive(fieldRelative, -omegaRadiansPerSecond, true, false);
    }).until(() -> {
      Boolean condition = isCloseEnoughToPose(targets[targets.length - 1].getReference(), endDistance, endAngle) &&
        waypointNumber == targets.length - 1;
      if(condition) SubsystemManager.getDrivebase().resetWaypointToPose();
      return condition;
    });
  }

  public void resetWaypointToPose() {
    waypointNumber = 0;
    rotationPID = new PIDController(SwerveDriveTuning.KP_ANGULAR_get(), 0, SwerveDriveTuning.KD_ANGULAR_get());
  }

  /* FEATURE DRIVE COMMANDS */

  public Command driveCommandWithAngle(Rotation2d angle) {
    return run(() -> {
      // Joystick translation
      double vx = RobotContainer.modifyAxis(SubsystemManager.getPSJoystick().getLeftY(), Constants.Operator.LEFT_Y_DEADBAND);
      double vy = RobotContainer.modifyAxis(SubsystemManager.getPSJoystick().getLeftX(), Constants.Operator.LEFT_X_DEADBAND);

      if(RobotContainer.teamColorIsBlue() == 1) {
        vx *= -1; vy *= -1; // Blue alliance joystick inversion
      }

      double slowdownFactor = SwerveDriveTuning.JOYSTICK_SLOWDOWN_FACTOR_get();

      if(SubsystemManager.getShooterState() == ShooterState.SHOOT || SubsystemManager.getShooterState() == ShooterState.SPEED_UP) {
        slowdownFactor = SwerveDriveTuning.JOYSTICK_SLOWDOWN_FACTOR_SHOOT_get();
      }

      Translation2d translation = SwerveMath.scaleTranslation(new Translation2d(
        vx * swerveDrive.getMaximumChassisVelocity(),
        vy * swerveDrive.getMaximumChassisVelocity()), slowdownFactor);
      // Rotation compute
      PIDController R = slickRotationPID;

      // Ensuring that the error is in the -180 ... +180 degree range
      double robotAngle = getPose().getRotation().getRadians();

      R.setSetpoint(angle.getRadians());
      double angleError = robotAngle - R.getSetpoint();

      if(angleError > Math.PI) robotAngle -= 2 * Math.PI;
      if(angleError < -Math.PI) robotAngle += 2 * Math.PI;

      double omegaRadiansPerSecond = R.calculate(robotAngle);

      swerveDrive.drive(translation, -omegaRadiansPerSecond, true, false);
    });
  }

  public Rotation2d determineHubAngle() {
    Translation2d translation = getHubTargetPose().getTranslation();
    if(RobotContainer.teamColorIsBlue() == 1) { // Blue
      return translation.minus(getPose().getTranslation()).getAngle();
    } else { // Red
      return translation.minus(getPose().getTranslation()).getAngle();
    }
  }

  public Rotation2d determineHubAngle(boolean virtual) {
    Translation2d translation = getHubTargetPose().getTranslation();
    if(virtual) translation = getVirtualTargetPose().getTranslation();
    if(RobotContainer.teamColorIsBlue() == 1) { // Blue
      return translation.minus(getPose().getTranslation()).getAngle();
    } else { // Red
      return translation.minus(getPose().getTranslation()).getAngle();
    }
  }

  public Pose2d getHubTargetPose() {
    Translation2d hubTranslation = RobotContainer.teamColorIsBlue() == 1
        ? new Translation2d(Constants.Field.HUB_X_BLUE, Constants.Field.HUB_Y_BLUE)
        : new Translation2d(Constants.Field.HUB_X_RED, Constants.Field.HUB_Y_RED);
    return new Pose2d(hubTranslation, hubTranslation.minus(getPose().getTranslation()).getAngle());
  }

  public Pose2d getVirtualTargetPose() {
    Translation2d hubTranslation = getHubTargetPose().getTranslation();
    
    double distance = calculateDistanceToHub();
    double flightTime = RobotContainer.Shooter.calculateFlightTime(distance);
    
    ChassisSpeeds fieldSpeeds = swerveDrive.getFieldVelocity(); 
    Translation2d ballOffset = new Translation2d(
        fieldSpeeds.vxMetersPerSecond * flightTime,
        fieldSpeeds.vyMetersPerSecond * flightTime
    );

    Translation2d virtualTarget = hubTranslation.minus(ballOffset);

    // Second round for higher accuracy
    double virtualDistance = getPose().getTranslation().getDistance(virtualTarget);
    double refinedFlightTime = RobotContainer.Shooter.calculateFlightTime(virtualDistance);

    virtualTarget = hubTranslation.minus(new Translation2d(
        fieldSpeeds.vxMetersPerSecond * refinedFlightTime,
        fieldSpeeds.vyMetersPerSecond * refinedFlightTime
    ));

    return new Pose2d(virtualTarget, virtualTarget.minus(getPose().getTranslation()).getAngle());
  }

  public double calculateDistanceToHub() { 
    Translation2d hubTranslation = getHubTargetPose().getTranslation();
    return getPose().getTranslation().getDistance(hubTranslation);
  }

  public double calculateDistanceToHub(boolean virtual) {
    Translation2d hubTranslation = getHubTargetPose().getTranslation();
    if(virtual) hubTranslation = getVirtualTargetPose().getTranslation();
    return getPose().getTranslation().getDistance(hubTranslation);
  }

  public Boolean isInNeutralZone() {
    double x = getPose().getX();
    return (x > Constants.Field.HUB_X_BLUE) && (x < Constants.Field.HUB_X_RED);
  }

  public Rotation2d calculateOptimalDiagonalRotation() {
    final Rotation2d option1 = Rotation2d.fromDegrees(45);
    final Rotation2d option2 = Rotation2d.fromDegrees(135);
    final Rotation2d option3 = Rotation2d.fromDegrees(225);
    final Rotation2d option4 = Rotation2d.fromDegrees(315);

    Rotation2d finalChoice = option1;

    if(isCloseEnoughToAngle(option2, 45)) finalChoice = option2;
    else if(isCloseEnoughToAngle(option3, 45)) finalChoice = option3;
    else if(isCloseEnoughToAngle(option4, 45)) finalChoice = option4;

    return finalChoice;
  }

  public static Pose2d getOutpostPose() {
    if(RobotContainer.teamColorIsBlue() == 1) {
      return new Pose2d(Constants.Field.OUTPOST_X_BLUE, Constants.Field.OUTPOST_Y_BLUE, Rotation2d.k180deg);
    } else {
      return new Pose2d(Constants.Field.OUTPOST_X_RED, Constants.Field.OUTPOST_Y_RED, Rotation2d.kZero);
    }
  }

  public static Pose2d getTowerPose() {
    if(RobotContainer.teamColorIsBlue() == 1) {
      return new Pose2d(Constants.Field.TOWER_X_BLUE, Constants.Field.TOWER_Y_BLUE, Rotation2d.kZero);
    } else {
      return new Pose2d(Constants.Field.TOWER_X_RED, Constants.Field.TOWER_Y_RED, Rotation2d.k180deg);
    }
  }

  public static Pose2d getDepotPose() {
    if(RobotContainer.teamColorIsBlue() == 1) {
      return new Pose2d(Constants.Field.DEPOT_X_BLUE, Constants.Field.DEPOT_Y_BLUE, Rotation2d.k180deg);
    } else {
      return new Pose2d(Constants.Field.DEPOT_X_RED, Constants.Field.DEPOT_Y_RED, Rotation2d.kZero);
    }
  }

  /* JOYSTICK DRIVE COMMAND */

  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX)
  {
    return run(() -> {
      Translation2d translation = SwerveMath.scaleTranslation(new Translation2d(
        translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
        translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()), SwerveDriveTuning.JOYSTICK_SLOWDOWN_FACTOR_get());
      double omegaRadiansPerSecond = Math.pow(angularRotationX.getAsDouble(), 3) * swerveDrive.getMaximumChassisAngularVelocity();
      
      // Make the robot move
      swerveDrive.drive(translation, omegaRadiansPerSecond,
                        true,
                        false);
    });
  }

  /* GETTER/SETTER FUNCTIONS */

  public static Command getAutonomousCommand(String pathName) {
    switch(pathName) {
      case "Collect (Red Left)":
        return new CollectAuto("red left");
      
      default:
      case "Collect (Red Right)":
        return new CollectAuto("red right");

      case "Collect (Blue Left)":
        return new CollectAuto("blue left");

      case "Collect (Blue Right)":
        return new CollectAuto("blue right");

      case "Shoot (Red Left)":
        return new ShootAuto("red left");

      case "Shoot (Red Right)":
        return new ShootAuto("red right");

      case "Shoot (Blue Left)":
        return new ShootAuto("blue left");

      case "Shoot (Blue Right)":
        return new ShootAuto("blue right");
    }
  }

  public void resetOdometry(Pose2d initialHolonomicPose)
  {
    swerveDrive.resetOdometry(initialHolonomicPose);
  }

  public Pose2d getSimulationPose() {
    return swerveDrive.getSimulationDriveTrainPose().get();
  }

  public void zeroGyro()
  {
    swerveDrive.zeroGyro();
  }

  public void zeroGyroWithAlliance()
  {
    if (RobotContainer.teamColorIsBlue() == 0)
    {
      zeroGyro();
      // Set the pose 180 degrees
      resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
    } else {
      zeroGyro();
    }
  }

  public void setMotorBrake(boolean brake)
  {
    swerveDrive.setMotorIdleMode(brake);
  }

  public Pose2d getPose() {
    if(Robot.isSimulation() && Constants.CurrentLimelightSimMode == LimelightSimMode.BASIC) return getSimulationPose();
    return swerveDrive.getPose();
  }

  public Rotation2d getHeading()
  {
    return getPose().getRotation();
  }

  public ChassisSpeeds getRobotVelocity()
  {
    return swerveDrive.getRobotVelocity();
  }

  public SwerveDriveConfiguration getSwerveDriveConfiguration()
  {
    return swerveDrive.swerveDriveConfiguration;
  }

  public void lock()
  {
    swerveDrive.lockPose();
  }

  /* SYSID ROUTINES */

  private SysIdRoutine driveSysIdRoutine;    
  private SysIdRoutine turnSysIdRoutine;    

  public Command driveSysIdQuasistatic(SysIdRoutine.Direction direction) {
    return driveSysIdRoutine.quasistatic(direction);
  }

  public Command driveSysIdDynamic(SysIdRoutine.Direction direction) {
    return driveSysIdRoutine.dynamic(direction);
  }

  public Command turnSysIdQuasistatic(SysIdRoutine.Direction direction) {
    return turnSysIdRoutine.quasistatic(direction);
  }

  public Command turnSysIdDynamic(SysIdRoutine.Direction direction) {
    return turnSysIdRoutine.dynamic(direction);
  }
}
