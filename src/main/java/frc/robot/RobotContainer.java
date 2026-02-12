package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.SysIdMode;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.Delivery.Conveyor.ConveyorSubsystem;
import frc.robot.subsystems.Delivery.Shooter.ShooterSubsystem;
import frc.robot.subsystems.IO.Real.CameraReal;
import frc.robot.subsystems.IO.Real.ConveyorReal;
import frc.robot.subsystems.IO.Real.IntakeReal;
import frc.robot.subsystems.IO.Real.ShooterReal;
// import frc.robot.subsystems.IO.Real.IntakeReal;
import frc.robot.subsystems.IO.Sim.CameraSim;
import frc.robot.subsystems.IO.Sim.ConveyorSim;
import frc.robot.subsystems.IO.Sim.IntakeSim;
import frc.robot.subsystems.IO.Sim.ShooterSim;
import frc.robot.subsystems.IO.Stub.CameraStub;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.SwerveDrive.SwerveSubsystem;

import java.util.Optional;

import com.ctre.phoenix6.SignalLogger;

// import org.littletonrobotics.junction.Logger;

public class RobotContainer {
  private static Command driveFieldOrientedAngularVelocity;

  public static ShooterSubsystem Shooter;
  public static IntakeSubsystem Intake;
  public static ConveyorSubsystem Conveyor;

  private static double periodicDt = 0.02;

  public static void init() {
    switch(Constants.CurrentMode) {
      case REAL:
        Limelight.init(new CameraReal());
        Shooter = new ShooterSubsystem(new ShooterReal());
        Intake = new IntakeSubsystem(new IntakeReal());
        Conveyor = new ConveyorSubsystem(new ConveyorReal());
        break;

      case REPLAY: 
        Limelight.init(new CameraStub());
        Shooter = new ShooterSubsystem(new ShooterReal());
        Intake = new IntakeSubsystem(new IntakeReal());
        Conveyor = new ConveyorSubsystem(new ConveyorReal());
        break;

      case SIM:
        Limelight.init(new CameraSim());
        Shooter = new ShooterSubsystem(new ShooterSim(periodicDt));
        Intake = new IntakeSubsystem(new IntakeSim(periodicDt));
        Conveyor = new ConveyorSubsystem(new ConveyorSim(periodicDt));
        break;
    }

    SubsystemManager.init();

    configureBindings();
  }

  private static void configureBindings() {
    SubsystemManager.getPSJoystick().PS().onTrue((Commands.runOnce(SubsystemManager.getDrivebase()::zeroGyroWithAlliance)));
      // to use in case the rotation is messed up

    if(Constants.CurrentSysIdMode == SysIdMode.ACTIVE) {
      SubsystemManager.getPSJoystick().L1().onTrue(Commands.runOnce(() -> SignalLogger.start()));
      SubsystemManager.getPSJoystick().R1().onTrue(Commands.runOnce(() -> SignalLogger.stop()));

      // Conveyor SysId [General subsystem template]

      // SubsystemManager.getPSJoystick().circle().whileTrue(Conveyor.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
      // SubsystemManager.getPSJoystick().square().whileTrue(Conveyor.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
      // SubsystemManager.getPSJoystick().triangle().whileTrue(Conveyor.sysIdDynamic(SysIdRoutine.Direction.kForward));
      // SubsystemManager.getPSJoystick().cross().whileTrue(Conveyor.sysIdDynamic(SysIdRoutine.Direction.kReverse));
      
      // Swerves | Drive SysId

      SubsystemManager.getPSJoystick().circle().whileTrue(SubsystemManager.getDrivebase().driveSysIdQuasistatic(SysIdRoutine.Direction.kForward));
      SubsystemManager.getPSJoystick().square().whileTrue(SubsystemManager.getDrivebase().driveSysIdQuasistatic(SysIdRoutine.Direction.kReverse));
      SubsystemManager.getPSJoystick().triangle().whileTrue(SubsystemManager.getDrivebase().driveSysIdDynamic(SysIdRoutine.Direction.kForward));
      SubsystemManager.getPSJoystick().cross().whileTrue(SubsystemManager.getDrivebase().driveSysIdDynamic(SysIdRoutine.Direction.kReverse));

      // Swerves | Turn SysId

      // SubsystemManager.getPSJoystick().circle().whileTrue(SubsystemManager.getDrivebase().turnSysIdQuasistatic(SysIdRoutine.Direction.kForward));
      // SubsystemManager.getPSJoystick().square().whileTrue(SubsystemManager.getDrivebase().turnSysIdQuasistatic(SysIdRoutine.Direction.kReverse));
      // SubsystemManager.getPSJoystick().triangle().whileTrue(SubsystemManager.getDrivebase().turnSysIdDynamic(SysIdRoutine.Direction.kForward));
      // SubsystemManager.getPSJoystick().cross().whileTrue(SubsystemManager.getDrivebase().turnSysIdDynamic(SysIdRoutine.Direction.kReverse));
    }
  }

  public static void configureDriveCommand() {
    if (teamColorIsBlue() == 0) {
      SubsystemManager.getDrivebase().zeroGyro();
      SubsystemManager.getDrivebase().resetOdometry(new Pose2d(SubsystemManager.getDrivebase().getPose().getTranslation(), Rotation2d.fromDegrees(180)));
      driveFieldOrientedAngularVelocity = SubsystemManager.getDrivebase().driveCommand(
        () -> (modifyAxis(SubsystemManager.getPSJoystick().getLeftY(), Constants.Operator.LEFT_Y_DEADBAND)),
        () -> (modifyAxis(SubsystemManager.getPSJoystick().getLeftX(), Constants.Operator.LEFT_X_DEADBAND)),
        () -> -modifyAxis(SubsystemManager.getPSJoystick().getRightX(), Constants.Operator.RIGHT_X_DEADBAND));
    } else if(teamColorIsBlue() == 1) {
      driveFieldOrientedAngularVelocity = SubsystemManager.getDrivebase().driveCommand(
        () -> (modifyAxis(-SubsystemManager.getPSJoystick().getLeftY(), Constants.Operator.LEFT_Y_DEADBAND)),
        () -> (modifyAxis(-SubsystemManager.getPSJoystick().getLeftX(), Constants.Operator.LEFT_X_DEADBAND)),
        () -> modifyAxis(SubsystemManager.getPSJoystick().getRightX(), Constants.Operator.RIGHT_X_DEADBAND));
    }
    
    SubsystemManager.setDefaultCommand(driveFieldOrientedAngularVelocity);
  }

  public static Command getAutonomousCommand() {
    return SwerveSubsystem.getAutonomousCommand(Dashboard.getSelectedAutonomy());
  }

  public static void setMotorBrake(boolean brake) {
    SubsystemManager.getDrivebase().setMotorBrake(brake);
  }

  public static int teamColorIsBlue() {
    try {
      Optional<Alliance> color = DriverStation.getAlliance();
	    return color.get() == DriverStation.Alliance.Blue ? 1 : 0;
    } catch (Exception e) {
      return -1;
    }
  }

  public static double modifyAxis(double joystickInput, double deadband) {
    double deadbandValue = MathUtil.applyDeadband(joystickInput, deadband);
    double scaled_input = 1 - (1 - Math.abs(deadbandValue)) * (1 - 0.1);
    return deadbandValue * scaled_input;
  }
}
