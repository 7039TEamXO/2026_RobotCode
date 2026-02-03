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
// import frc.robot.RobotContainer;
import frc.robot.Constants.Mode;
import frc.robot.Constants.SysIdMode;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.SwerveDrive.SwerveDriveTuning;
import frc.robot.subsystems.SwerveDrive.SwerveSubsystem;
public class SubsystemManager {
    private static final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve/falcon"));

    private static final CommandPS4Controller ps4Joystick = new CommandPS4Controller(0);
    private static final PS4Controller psControllerHID = ps4Joystick.getHID();


    private static IntakeState intakeState;
    private static IntakeState lastIntakeState;

    public static void init() {

        intakeState = IntakeState.INACTIVE;
        lastIntakeState = intakeState;
    }

    private static Command autoDrive;

    private static boolean isInAutoDrive;

    private static void scheduleAutoDrive() {
        if(psControllerHID.getR1Button()) { // Test drive
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

    private static void computeInputStates() {
        intakeState = psControllerHID.getCircleButton() ? IntakeState.ACTIVE :
                      psControllerHID.getSquareButton() ? IntakeState.DEPLETE :
                      psControllerHID.getCrossButton() ? IntakeState.INACTIVE :
                      lastIntakeState;
    }

    /* ************************* */

    public static void operate(boolean onAuto) { 
        if(Constants.CurrentMode != Mode.REAL) simulationPeriodic();

        if (!onAuto) {
            if(Constants.CurrentSysIdMode == SysIdMode.INACTIVE) {
                scheduleAutoDrive();
            }

            computeInputStates();
        }

        if(Constants.CurrentSysIdMode == SysIdMode.INACTIVE) {
            // ...
        }

        // lastState = state;
        lastIntakeState = intakeState;
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

