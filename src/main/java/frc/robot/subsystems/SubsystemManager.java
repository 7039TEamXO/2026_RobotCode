package frc.robot.subsystems;

import java.io.File;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.Mode;
import frc.robot.Constants.SysIdMode;
import frc.robot.subsystems.Delivery.Conveyor.ConveyorState;
import frc.robot.subsystems.Delivery.Shooter.ShooterState;
import frc.robot.subsystems.Delivery.Shooter.ShooterTuning;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Intake.StorageState;
import frc.robot.subsystems.SwerveDrive.SwerveDriveTuning;
import frc.robot.subsystems.SwerveDrive.SwerveSubsystem;
public class SubsystemManager {
    private static final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve/falcon"));

    private static final CommandPS4Controller ps4Joystick = new CommandPS4Controller(0);
    private static final PS4Controller psControllerHID = ps4Joystick.getHID();
    private static IntakeState intakeState;
    private static StorageState storageState;
    private static ShooterState shooterState;
    private static ConveyorState conveyorState;
    private static Boolean conveyorUnjam;
    private static Boolean hubAlignment;

    public static void init() {
        intakeState = IntakeState.INACTIVE;
        storageState = StorageState.IN;
        shooterState = ShooterState.IDLE;
        conveyorState = ConveyorState.STOP;
        conveyorUnjam = false;
        hubAlignment = false;
    }

    private static Command autoDrive;
    private static boolean isInAutoDrive;

    private static void scheduleAutoDrive() {
        // TO-DO: Check if in ramp zone
        if(psControllerHID.getPOV() == 180) { // Outpost
            hubAlignment = false;
            autoDrive = drivebase.driveToPose(SwerveSubsystem.getOutpostPose(), false);
            CommandScheduler.getInstance().schedule(autoDrive);
        } else if(psControllerHID.getR2Button()) { // Tower
            hubAlignment = false;
            autoDrive = drivebase.driveToPose(SwerveSubsystem.getTowerPose(), false);
            CommandScheduler.getInstance().schedule(autoDrive);
        } else if(psControllerHID.getR3Button()) { // Depot / Test
            hubAlignment = false; 
            // autoDrive = drivebase.driveToPose(new Pose2d(SwerveDriveTuning.TEST_DRIVE_X_get(), SwerveDriveTuning.TEST_DRIVE_Y_get(),
            //     new Rotation2d(Math.toRadians(SwerveDriveTuning.TEST_DRIVE_ANGLE_get()))));
            autoDrive = drivebase.driveToPose(SwerveSubsystem.getDepotPose(), false);
            CommandScheduler.getInstance().schedule(autoDrive);
        } else if(psControllerHID.getL3Button()) { // Ramp Alignment (Lower priority)
            autoDrive = drivebase.driveCommandWithAngle(drivebase.calculateOptimalDiagonalRotation());
            CommandScheduler.getInstance().schedule(autoDrive);
        } else if(hubAlignment) { // Hub Alignment (Lowest priority)
            autoDrive = drivebase.driveCommandWithAngle(drivebase.determineHubAngle());
            CommandScheduler.getInstance().schedule(autoDrive);
        } else {
            if(autoDrive != null) {
                CommandScheduler.getInstance().cancel(autoDrive);
                autoDrive = null;
            }
        }
        isInAutoDrive = autoDrive != null;
    }

    private static void calculateButtonStates() {
        /* INTAKE */
        // 1. Full Stop (Square) takes priority
        if (psControllerHID.getSquareButton()) {
            intakeState = IntakeState.INACTIVE;
            storageState = StorageState.IN;
            shooterState = ShooterState.IDLE;
        } 
        // 2. Main Intake Routine (Cross)
        else if (psControllerHID.getCrossButton()) {
            intakeState = IntakeState.ACTIVE;
            storageState = StorageState.OUT;
            shooterState = ShooterState.IDLE;
        } 
        // 3. Deplete Routine (Circle)
        else if (psControllerHID.getCircleButton()) {
            intakeState = IntakeState.DEPLETE;
            storageState = StorageState.OUT;
            shooterState = ShooterState.DEPLETE;
        } 
        // 4. Manual Stop (Triangle)
        else if (psControllerHID.getTriangleButton()) {
            intakeState = IntakeState.INACTIVE;
            storageState = StorageState.OUT;
            shooterState = ShooterState.IDLE;
        }
        // 5. Help Shoot (L2)
        else if (psControllerHID.getL2Button()) {
            storageState = StorageState.HELP_SHOOT;
            intakeState = IntakeState.HELP_SHOOT;
        }

        /* SHOOTER */
        shooterState = psControllerHID.getL2Button() ? ShooterState.SHOOT :
                        psControllerHID.getL1Button() ? ShooterState.SPEED_UP :
                        psControllerHID.getR1Button() ? ShooterState.IDLE : 
                        shooterState; 

        // Cancel Help Shoot once it's no longer needed
        if(shooterState != ShooterState.SHOOT) {
            if(storageState == StorageState.HELP_SHOOT) storageState = StorageState.OUT;
            if(intakeState == IntakeState.HELP_SHOOT) intakeState = IntakeState.INACTIVE;
        }


        conveyorUnjam = psControllerHID.getPOV() == 0;
        if (psControllerHID.getPOV() == 90) {
            hubAlignment = true;
        } else if (psControllerHID.getPOV() == 270) {
            hubAlignment = false;
        }
        if (shooterState == ShooterState.SHOOT) {
            hubAlignment = true;
        }
    }

    /* ************************* */

    public static void operate(boolean onAuto) { 
        if(Constants.CurrentMode != Mode.REAL) simulationPeriodic();
        if(Constants.CurrentSysIdMode == SysIdMode.INACTIVE) {
            if (!onAuto) {
                scheduleAutoDrive();
                calculateButtonStates();
            }

            RobotContainer.Intake.operate(intakeState, storageState);

            System.out.println(drivebase.calculateDistanceToHub());
            Boolean atShootSpeed = Math.abs(RobotContainer.Shooter.getInputs().ShootMaster_Velocity - RobotContainer.Shooter.calculateShooterVelocity()) < ShooterTuning.SHOOT_VELOCITY_TOLERANCE_get();
            Boolean atShootAngle = drivebase.isCloseEnoughToAngle(drivebase.determineHubAngle(), SwerveDriveTuning.PRECISE_ANGLE_ERROR_CAP_get());
            atShootAngle = true;

            conveyorState = ConveyorState.STOP;

            switch (shooterState) {
                case SHOOT:
                    if (!atShootAngle && (!atShootSpeed || !Robot.isAuto())) {
                        shooterState = ShooterState.SPEED_UP;
                        conveyorState = ConveyorState.STOP;
                    } else {
                        shooterState = ShooterState.SHOOT;
                        conveyorState = ConveyorState.GUIDE;
                    }
                    break;

                case INTAKE_HELP:
                    conveyorState = ConveyorState.INTAKE_HELP;
                    break;

                case DEPLETE:
                    conveyorState = ConveyorState.DEPLETE;
                    break;

                case SPEED_UP:
                    conveyorState = ConveyorState.STOP;
                    break;

                default:
                case IDLE:
                    conveyorState = ConveyorState.STOP;
                    break;
            }

            // Unjamming has the highest priority
            if(conveyorUnjam) {
                conveyorState = ConveyorState.UNJAM;
                if(shooterState == ShooterState.SHOOT) shooterState = ShooterState.SPEED_UP;
            }

            RobotContainer.Conveyor.operate(conveyorState);
            RobotContainer.Shooter.operate(shooterState);
        }
    }

    public static Command operateAuto(ShooterState chosenShooterState, IntakeState chosenIntakeState, StorageState chosenStorageState) {
        return new InstantCommand(() -> {
            shooterState = chosenShooterState == null ? shooterState : chosenShooterState;
            intakeState = chosenIntakeState == null ? intakeState : chosenIntakeState;
            storageState = chosenStorageState == null ? storageState : chosenStorageState;
        });
    }

    private static void simulationPeriodic() {
        RobotContainer.Conveyor.simulationPeriodic();
        RobotContainer.Intake.simulationPeriodic();
        RobotContainer.Shooter.simulationPeriodic();
    }
    
    public static SwerveSubsystem getDrivebase() {
        return drivebase;
    }

    public static void setDefaultCommand(Command defaultCommand) {
        drivebase.setDefaultCommand(defaultCommand);
    }

    public static CommandPS4Controller getPSJoystick() {
        return ps4Joystick;
    } 

    public static boolean isInAutoDrive() {
        return isInAutoDrive;
    }

    public static ShooterState getShooterState() {
        return shooterState;
    }

    public static ConveyorState getConveyorState() {
        return conveyorState;
    }

    public static IntakeState getIntakeState() {
        return intakeState;
    }

    public static StorageState getStorageState() {
        return storageState;
    }
}

