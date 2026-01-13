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
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Limelight;
import frc.robot.Constants.TuningMode;
import frc.robot.commands.TestAuto;
import frc.robot.subsystems.SubsystemManager;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import java.io.File;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {
  private SwerveDrive swerveDrive;
  // private SwerveDrive swerveDriveOdometry; // Tuning

  public SwerveSubsystem(File directory) {
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    try {
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
      rotationPID.setPID(SwerveDriveTuning.KP_ANGULAR_get(), SwerveDriveTuning.KI_ANGULAR_get(), SwerveDriveTuning.KD_ANGULAR_get());
    }

    // if(isAuto) filter by isRobotVBelowOne too
    Object[] tuple = Limelight.update();
    if (tuple != null && (double)tuple[3] > 0.2) {
      swerveDrive.addVisionMeasurement((Pose2d)tuple[0], (double)tuple[1]);
    }

    swerveDrive.updateOdometry();
    // swerveDriveOdometry.updateOdometry(); // Tuning

    Logger.recordOutput("Odometry/Pose", getPose());
    if(Constants.CurrentMode != frc.robot.Constants.Mode.REAL) Logger.recordOutput("Odometry/RealPose", getSimulationPose());
    if(!SubsystemManager.isDriveToPoseActive()) {
      Logger.recordOutput("Odometry/RequestedPose", new Pose2d(Double.MAX_VALUE, Double.MAX_VALUE, new Rotation2d()));
    }
    // Logger.recordOutput("Odometry/BasicPose", swerveDriveOdometry.getPose()); // Tuning
  }

  @Override
  public void simulationPeriodic() {}

  public boolean isCloseEnoughToPose(Pose2d pose) {
    return Math.abs(getPose().getX() - pose.getX()) < SwerveDriveTuning.CLOSE_DISTANCE_ERROR_CAP_get() &&
      Math.abs(getPose().getY() - pose.getY()) < SwerveDriveTuning.CLOSE_DISTANCE_ERROR_CAP_get();
  }
  
  public Command getAutonomousCommand(String pathName) {
    switch(pathName) {
      default:
      case "SimpleAuto":
        return new TestAuto();
    }
  }

  private PIDController rotationPID = new PIDController(SwerveDriveConstants.KP_ANGULAR, SwerveDriveConstants.KI_ANGULAR, SwerveDriveConstants.KD_ANGULAR);

  public Command driveToPose(Pose2d pose) {
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

      swerveDrive.drive(fieldRelative, omegaRadiansPerSecond, true, false);
    });
  }

  private Integer waypointNumber = 0;

  public Command waypointToPose(APTarget[] targets) {
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

      // Hoppin onto the next waypoint
      if(isCloseEnoughToPose(targets[waypointNumber].getReference())) {
        if(waypointNumber < targets.length - 1) waypointNumber++; 
      }

      swerveDrive.drive(fieldRelative, omegaRadiansPerSecond, true, false);
    });
  }

  public void resetWaypointToPose() {
    waypointNumber = 0;
  }

  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,
      DoubleSupplier headingY) {
    // swerveDrive.setHeadingCorrection(true); // Normally you would want heading correction for this kind of control.
    return run(() -> {
      Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d(translationX.getAsDouble(),
          translationY.getAsDouble()), 0.8);

      ChassisSpeeds speeds = swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(),
          headingX.getAsDouble(),
          headingY.getAsDouble(),
          swerveDrive.getOdometryHeading().getRadians(),
          swerveDrive.getMaximumChassisVelocity());

      // Make the robot move
      driveFieldOriented(speeds);
    });
  }

  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX)
  {
    return run(() -> {
      Translation2d translation = SwerveMath.scaleTranslation(new Translation2d(
        translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
        translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()), 0.8);
      double omegaRadiansPerSecond = Math.pow(angularRotationX.getAsDouble(), 3) * swerveDrive.getMaximumChassisAngularVelocity();
      
      // Make the robot move
      swerveDrive.drive(translation, omegaRadiansPerSecond,
                        true,
                        false);
    });
  }

  public void drive(Translation2d translation, double rotation, boolean fieldRelative)
  {
    swerveDrive.drive(translation,
                      rotation,
                      fieldRelative,
                      false); // Open loop is disabled since it shouldn't be used most of the time.
  }

  public void driveFieldOriented(ChassisSpeeds velocity)
  {
    swerveDrive.driveFieldOriented(velocity);
  }

  public void drive(ChassisSpeeds velocity)
  {
    swerveDrive.drive(velocity);
  }

  public SwerveDriveKinematics getKinematics()
  {
    return swerveDrive.kinematics;
  }

  public void resetOdometry(Pose2d initialHolonomicPose)
  {
    swerveDrive.resetOdometry(initialHolonomicPose);
  }

  public Pose2d getSimulationPose() {
    return swerveDrive.getSimulationDriveTrainPose().get();
  }

  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds)
  {
    swerveDrive.setChassisSpeeds(chassisSpeeds);
  }

  public void zeroGyro()
  {
    swerveDrive.zeroGyro();
  }

  private static boolean isRedAlliance()
  {
    var alliance = DriverStation.getAlliance();
    return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
  }

  public void zeroGyroWithAlliance()
  {
    if (isRedAlliance())
    {
      zeroGyro();
      // Set the pose 180 degrees
      resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
    } else
    {
      zeroGyro();
    }
  }

  public void setMotorBrake(boolean brake)
  {
    swerveDrive.setMotorIdleMode(brake);
  }

  public Pose2d getPose() {
    // return swerveDrive.getPose();
    return getSimulationPose();
  }

  public Rotation2d getHeading()
  {
    return getPose().getRotation();
  }
  
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY)
  {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));
    return swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
                                                        scaledInputs.getY(),
                                                        headingX,
                                                        headingY,
                                                        getHeading().getRadians(),
                                                        SwerveDriveConstants.MAX_SPEED);
  }

  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle)
  {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));

    return swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
                                                        scaledInputs.getY(),
                                                        angle.getRadians(),
                                                        getHeading().getRadians(),
                                                        SwerveDriveConstants.MAX_SPEED);
  }

  public ChassisSpeeds getFieldVelocity()
  {
    return swerveDrive.getFieldVelocity();
  }

  public ChassisSpeeds getRobotVelocity()
  {
    return swerveDrive.getRobotVelocity();
  }

  public SwerveController getSwerveController()
  {
    return swerveDrive.swerveController;
  }

  public SwerveDriveConfiguration getSwerveDriveConfiguration()
  {
    return swerveDrive.swerveDriveConfiguration;
  }

  public void lock()
  {
    swerveDrive.lockPose();
  }

  public Rotation2d getPitch()
  {
    return swerveDrive.getPitch();
  }

  // SysId routines //

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
