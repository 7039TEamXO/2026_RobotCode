package frc.robot.subsystems;

import java.io.File;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
// import frc.robot.RobotContainer;
import frc.robot.Constants.Mode;
import frc.robot.Constants.SysIdMode;
import frc.robot.subsystems.Delivery.Conveyor.ConveyorState;
import frc.robot.subsystems.Delivery.Shooter.ShooterConstants;
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

    private static Boolean conveyorUnjam;
    private static Boolean hubAlignment;

    public static void init() {
        intakeState = IntakeState.INACTIVE;
        storageState = StorageState.IN;
        shooterState = ShooterState.IDLE;
        conveyorUnjam = false;
        hubAlignment = false;
    }

    private static Command autoDrive;

    private static boolean isInAutoDrive;

    private static void scheduleAutoDrive() {
        if(psControllerHID.getL3Button()) { // Test drive
            autoDrive = drivebase.driveToPose(new Pose2d(SwerveDriveTuning.TEST_DRIVE_X_get(), SwerveDriveTuning.TEST_DRIVE_Y_get(),
                new Rotation2d(Math.toRadians(SwerveDriveTuning.TEST_DRIVE_ANGLE_get()))));
            CommandScheduler.getInstance().schedule(autoDrive);
        } else {
            drivebase.resetWaypointToPose();
            if(autoDrive != null) {
                CommandScheduler.getInstance().cancel(autoDrive);
                autoDrive = null;
            }
        }
        isInAutoDrive = autoDrive != null;
    }

    private static void calculateButtonStates() {
        /* INTAKE */

        if(psControllerHID.getSquareButton()) {
            intakeState = IntakeState.INACTIVE;

            storageState = StorageState.IN;
        } else {
            intakeState = psControllerHID.getCrossButton() ? IntakeState.ACTIVE :
                psControllerHID.getCircleButton() ? IntakeState.DEPLETE :
                psControllerHID.getL2Button() ? IntakeState.INACTIVE :
                intakeState;

            if(psControllerHID.getCrossButton() || psControllerHID.getCircleButton()) {
                storageState = StorageState.OUT;
            }
        }

        /* SHOOTER */

        shooterState = psControllerHID.getL1Button() ? ShooterState.SPEED_UP :
                        psControllerHID.getL2Button() ? ShooterState.SHOOT :
                        psControllerHID.getTriangleButton() ? ShooterState.IDLE : 
                        shooterState;

        conveyorUnjam = psControllerHID.getPOV() == 0;
        if(psControllerHID.getPOV() == 90) {
            hubAlignment = true;
        } else if(psControllerHID.getPOV() == 270) {
            hubAlignment = false;
        }

        if(shooterState == ShooterState.SHOOT) {
            hubAlignment = true; 
        }
    }

    /* ************************* */

    public static void operate(boolean onAuto) { 
        if(Constants.CurrentMode != Mode.REAL) simulationPeriodic();

        if (!onAuto) {
            if(Constants.CurrentSysIdMode == SysIdMode.INACTIVE) {
                scheduleAutoDrive();
                calculateButtonStates();
            }
        }

        if(Constants.CurrentSysIdMode == SysIdMode.INACTIVE) {
            RobotContainer.Intake.operate(intakeState, storageState);

            Boolean atShootSpeed = Math.abs(RobotContainer.Shooter.getInputs().ShootMaster_Velocity - ShooterTuning.Shoot_Voltage_Fire_get()) < ShooterTuning.Shoot_Voltage_Tolerance_get();
            Boolean atShootAngle = drivebase.isCloseEnoughToAngle(determineHubAngle(), SwerveDriveTuning.PRECISE_ANGLE_ERROR_CAP_get());

            switch(shooterState) {
                case SHOOT:
                    
                    break;

                case SPEED_UP:

                    break;

                default:
                case IDLE:
                    
                    break;
            }
        }
    }

    private static Rotation2d determineHubAngle() {
        return Rotation2d.kZero;
    }

    private static void simulationPeriodic() {
        // ...
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
}

