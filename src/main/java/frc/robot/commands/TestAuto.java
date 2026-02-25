package frc.robot.commands;

import com.therekrab.autopilot.APTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.Constants;
import frc.robot.Limelight;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.Delivery.Shooter.ShooterState;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Intake.StorageState;
import frc.robot.subsystems.SwerveDrive.SwerveSubsystem;
public class TestAuto extends SequentialCommandGroup {
    private Pose2d START;
    private Pose2d RAMP_ENTRANCE;
    private Rotation2d RAMP_ENTRANCE_EA; // EA = Entry Angle
    private Pose2d RAMP_EXIT;

    private Pose2d STEAL_START;
    private Rotation2d STEAL_START_EA;
    private Pose2d SHOOT_FIRST_ENTRANCE;
    private Pose2d SHOOT_FIRST;
    private Pose2d STEAL_END;

    private Pose2d RAMP_NEXT_ENTRANCE;
    private Pose2d RAMP_NEXT_EXIT;
    private Pose2d FINAL;

    private Rotation2d ToRedLeft(Rotation2d angle) {
        return angle.times(-1);
    }

    private Rotation2d ToBlueRight(Rotation2d angle) {
        return angle.plus(Rotation2d.k180deg).times(-1);
    }

    private Rotation2d ToBlueLeft(Rotation2d angle) {
        return angle.plus(Rotation2d.k180deg);
    }

    private Pose2d ToRedLeft(Pose2d pose) {
        return new Pose2d(pose.getX(), Constants.Field.FIELD_HEIGHT - pose.getY(), ToRedLeft(pose.getRotation()));
    }

    private Pose2d ToBlueRight(Pose2d pose) {
        return new Pose2d(Constants.Field.FIELD_WIDTH - pose.getX(), pose.getY(), ToBlueRight(pose.getRotation()));
    }

    private Pose2d ToBlueLeft(Pose2d pose) {
        return new Pose2d(Constants.Field.FIELD_WIDTH - pose.getX(), Constants.Field.FIELD_HEIGHT - pose.getY(), ToBlueLeft(pose.getRotation()));
    }

    public TestAuto(String orientation) {
        // default is red left
        START = new Pose2d(13, 3.5, Rotation2d.kZero);
        RAMP_ENTRANCE = new Pose2d(13.2, 5.2, Rotation2d.fromDegrees(-45));
        RAMP_ENTRANCE_EA = Rotation2d.k180deg;
        RAMP_EXIT = new Pose2d(10.6, 5.9, Rotation2d.fromDegrees(-45));
        STEAL_START = new Pose2d(8.725, 6.8, Rotation2d.kCW_90deg);
        STEAL_START_EA = Rotation2d.k180deg;
        SHOOT_FIRST_ENTRANCE = new Pose2d(8.725, 4.5, Rotation2d.kCW_90deg);
        SHOOT_FIRST = new Pose2d(8.725, 4.5, Rotation2d.fromDegrees(30));
        STEAL_END = new Pose2d(8.725, 2.3, Rotation2d.kCW_90deg);
        RAMP_NEXT_ENTRANCE = new Pose2d(10.6, 2.3, Rotation2d.fromDegrees(-45));
        RAMP_NEXT_EXIT = new Pose2d(13.2, 2.8, Rotation2d.fromDegrees(-45));
        FINAL = SwerveSubsystem.getTowerPose();

        switch(orientation) {
            default:
            case "red right":
                // Do nothing
                break;
            
            case "red left":
                START = ToRedLeft(START);
                RAMP_ENTRANCE = ToRedLeft(RAMP_ENTRANCE);
                RAMP_ENTRANCE_EA = ToRedLeft(RAMP_ENTRANCE_EA);
                RAMP_EXIT = ToRedLeft(RAMP_EXIT);
                STEAL_START = ToRedLeft(STEAL_START);
                STEAL_START_EA = ToRedLeft(STEAL_START_EA);
                SHOOT_FIRST_ENTRANCE = ToRedLeft(SHOOT_FIRST_ENTRANCE);
                SHOOT_FIRST = ToRedLeft(SHOOT_FIRST);
                STEAL_END = ToRedLeft(STEAL_END);
                RAMP_NEXT_ENTRANCE = ToRedLeft(RAMP_NEXT_ENTRANCE);
                RAMP_NEXT_EXIT = ToRedLeft(RAMP_NEXT_EXIT);
                break;
            
            case "blue right":
                START = ToBlueRight(START);
                RAMP_ENTRANCE = ToBlueRight(RAMP_ENTRANCE);
                RAMP_ENTRANCE_EA = ToBlueRight(RAMP_ENTRANCE_EA);
                RAMP_EXIT = ToBlueRight(RAMP_EXIT);
                STEAL_START = ToBlueRight(STEAL_START);
                STEAL_START_EA = ToBlueRight(STEAL_START_EA);
                SHOOT_FIRST_ENTRANCE = ToBlueRight(SHOOT_FIRST_ENTRANCE);
                SHOOT_FIRST = ToBlueRight(SHOOT_FIRST);
                STEAL_END = ToBlueRight(STEAL_END);
                RAMP_NEXT_ENTRANCE = ToBlueRight(RAMP_NEXT_ENTRANCE);
                RAMP_NEXT_EXIT = ToBlueRight(RAMP_NEXT_EXIT);
                break;

            case "blue left":
                START = ToBlueLeft(START);
                RAMP_ENTRANCE = ToBlueLeft(RAMP_ENTRANCE);
                RAMP_ENTRANCE_EA = ToBlueLeft(RAMP_ENTRANCE_EA);
                RAMP_EXIT = ToBlueLeft(RAMP_EXIT);
                STEAL_START = ToBlueLeft(STEAL_START);
                STEAL_START_EA = ToBlueLeft(STEAL_START_EA);
                SHOOT_FIRST_ENTRANCE = ToBlueLeft(SHOOT_FIRST_ENTRANCE);
                SHOOT_FIRST = ToBlueLeft(SHOOT_FIRST);
                STEAL_END = ToBlueLeft(STEAL_END);
                RAMP_NEXT_ENTRANCE = ToBlueLeft(RAMP_NEXT_ENTRANCE);
                RAMP_NEXT_EXIT = ToBlueLeft(RAMP_NEXT_EXIT);
                break;
        }

        setup();
    }

    private void setup() {
        addCommands(
            new InstantCommand(() -> {
                if(!Limelight.hasTarget() || Robot.isSimulation()) 
                    SubsystemManager.getDrivebase().resetOdometry(START);
            }),
            SubsystemManager.operateAuto(ShooterState.IDLE, IntakeState.INACTIVE, StorageState.IN),
            SubsystemManager.getDrivebase().waypointToPose(new APTarget[]{
                new APTarget(RAMP_ENTRANCE).withVelocity(1).withEntryAngle(RAMP_ENTRANCE_EA),
                new APTarget(RAMP_EXIT).withVelocity(2)
            }, 0.3, true),
            SubsystemManager.operateAuto(ShooterState.IDLE, IntakeState.ACTIVE, StorageState.OUT),
            SubsystemManager.getDrivebase().waypointToPose(new APTarget[]{
                new APTarget(STEAL_START).withVelocity(0).withEntryAngle(STEAL_START_EA),
                new APTarget(SHOOT_FIRST_ENTRANCE).withVelocity(0)
            }, 0.2, true),
            SubsystemManager.operateAuto(ShooterState.SPEED_UP, IntakeState.ACTIVE, StorageState.OUT),
            SubsystemManager.getDrivebase().driveToPose(new APTarget(SHOOT_FIRST).withVelocity(0), 0.1, 1),
            SubsystemManager.operateAuto(ShooterState.SHOOT, IntakeState.INACTIVE, StorageState.OUT),
            Commands.waitSeconds(3),
            SubsystemManager.operateAuto(ShooterState.IDLE, IntakeState.ACTIVE, StorageState.OUT),
            SubsystemManager.getDrivebase().driveToPose(new APTarget(SHOOT_FIRST_ENTRANCE).withVelocity(0), 0.1, 1),
            // velocity is zero for a better steal
            SubsystemManager.getDrivebase().driveToPose(new APTarget(STEAL_END).withVelocity(0), true), 
            SubsystemManager.operateAuto(ShooterState.IDLE, IntakeState.INACTIVE, StorageState.IN),
            SubsystemManager.getDrivebase().waypointToPose(new APTarget[] {
                new APTarget(RAMP_NEXT_ENTRANCE).withVelocity(2),
                new APTarget(RAMP_NEXT_EXIT).withVelocity(2),
                new APTarget(FINAL).withVelocity(0)
            }, 0.2, false)
        );
    }
}