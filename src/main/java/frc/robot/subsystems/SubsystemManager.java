package frc.robot.subsystems;

import java.io.File;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.Constants.Mode;
import frc.robot.Constants.SysIdMode;
import frc.robot.subsystems.SwerveDrive.SwerveDriveTuning;
import frc.robot.subsystems.SwerveDrive.SwerveSubsystem;
public class SubsystemManager {
    private static final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve/falcon"));

    private static final CommandPS4Controller ps4Joystick = new CommandPS4Controller(0);
    private static final PS4Controller psControllerHID = ps4Joystick.getHID();

    private static RobotState state;

    public static void init() {
        state = RobotState.TRAVEL;
    }

    private static Command testDrive;

    private static boolean isDriveToPoseActive;

    private static void scheduleDriveCommands() {
        if(psControllerHID.getR1Button()) {
            testDrive = drivebase.driveToPose(new Pose2d(SwerveDriveTuning.TEST_DRIVE_X_get(), SwerveDriveTuning.TEST_DRIVE_Y_get(),
                new Rotation2d(Math.toRadians(SwerveDriveTuning.TEST_DRIVE_ANGLE_get()))));
            CommandScheduler.getInstance().schedule(testDrive);
        } else {
            if(testDrive != null) {
                CommandScheduler.getInstance().cancel(testDrive);
                testDrive = null;
            }
        }
    }

    /* ************************* */

    public static void operate(boolean onAuto) { 
        if(Constants.CurrentMode != Mode.REAL) simulationPeriodic();

        isDriveToPoseActive = testDrive != null;
        
        if (!onAuto) {
            if(Constants.CurrentSysIdMode == SysIdMode.INACTIVE) scheduleDriveCommands();
        }
    }
    
    public static Command operateAuto(RobotState chosenState) {
        return new InstantCommand(() -> {
            state = chosenState == null ? state : chosenState;
        });
    }

    private static void simulationPeriodic() {}
    
    public static SwerveSubsystem getDrivebase() {
        return drivebase;
    }

    public static void setDefaultCommand(Command defaultCommand) {
        drivebase.setDefaultCommand(defaultCommand);
    }

    public static CommandPS4Controller getPSJoystick() {
        return ps4Joystick;
    } 

    public static RobotState getRobotState() {
        return state;
    }

    public static boolean isDriveToPoseActive() {
        return isDriveToPoseActive;
    }

    public static void setStateToDefault() {
        state = RobotState.TRAVEL;
    }
}