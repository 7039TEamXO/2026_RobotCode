package frc.robot.commands;

import com.therekrab.autopilot.APTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.Delivery.Shooter.ShooterState;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Intake.StorageState;

public class CollectAuto extends SequentialCommandGroup {
    private Pose2d RAMP_START;
    private Pose2d RAMP_CROSS;

    private Pose2d STEAL_START;
    private Rotation2d STEAL_START_EA; // Entry angle
    private Pose2d STEAL_CENTER;
    private Rotation2d STEAL_CENTER_EA;

    private Pose2d NEXT_STEAL_CENTER;
    private Pose2d NEXT_STEAL_END;

    private Pose2d TROUGH_FIRST;
    private Pose2d TROUGH_NEXT;
    private Rotation2d TROUGH_EA;

    private Pose2d END_START;
    private Pose2d END_HUB;

    private Pose2d RAMP_FINISH;

    private Rotation2d ToRedLeft(Rotation2d angle) {
        return angle.times(-1);
    }

    private Rotation2d ToBlueLeft(Rotation2d angle) {
        return angle.plus(Rotation2d.k180deg).times(-1);
    }

    private Rotation2d ToBlueRight(Rotation2d angle) {
        return angle.plus(Rotation2d.k180deg);
    }

    private Pose2d ToRedLeft(Pose2d pose) {
        return new Pose2d(pose.getX(), Constants.Field.FIELD_HEIGHT - pose.getY(), ToRedLeft(pose.getRotation()));
    }

    private Pose2d ToBlueLeft(Pose2d pose) {
        return new Pose2d(Constants.Field.FIELD_WIDTH - pose.getX(), pose.getY(), ToBlueLeft(pose.getRotation()));
    }

    private Pose2d ToBlueRight(Pose2d pose) {
        return new Pose2d(Constants.Field.FIELD_WIDTH - pose.getX(), Constants.Field.FIELD_HEIGHT - pose.getY(), ToBlueRight(pose.getRotation()));
    }

    public CollectAuto(String orientation) {
        // Default is "red right"
        RAMP_START = new Pose2d(13.2, 5.4, Rotation2d.kCW_90deg);
        RAMP_CROSS = new Pose2d(10.8, 5.8, Rotation2d.kCW_90deg);
        STEAL_START = new Pose2d(9.5, 7, Rotation2d.kCW_90deg);
        STEAL_START_EA = Rotation2d.k180deg;
        STEAL_CENTER = new Pose2d(8.725, 4, Rotation2d.kCW_90deg);
        STEAL_CENTER_EA = Rotation2d.kCW_90deg;
        NEXT_STEAL_CENTER = new Pose2d(10.5, 4, Rotation2d.kCW_90deg);
        NEXT_STEAL_END = new Pose2d(10.5, 2, Rotation2d.kCW_90deg);
        TROUGH_FIRST = new Pose2d(11, 7.4, Rotation2d.kZero);
        TROUGH_NEXT = new Pose2d(11, 0.6, Rotation2d.kZero);
        TROUGH_EA = Rotation2d.kZero;
        END_START = new Pose2d(10, 2, Rotation2d.kCCW_90deg);
        END_HUB = new Pose2d(10, 4, Rotation2d.kCCW_90deg);
        RAMP_FINISH = new Pose2d(10.8, 5.4, Rotation2d.kCCW_90deg);

        switch(orientation) {
            default:
            case "red right":
                // Do nothing
                break;
            
            case "red left":
                RAMP_START = ToRedLeft(RAMP_START);
                RAMP_CROSS = ToRedLeft(RAMP_CROSS);
                STEAL_START = ToRedLeft(STEAL_START);
                STEAL_START_EA = ToRedLeft(STEAL_START_EA);
                STEAL_CENTER = ToRedLeft(STEAL_CENTER);
                STEAL_CENTER_EA = ToRedLeft(STEAL_CENTER_EA);
                NEXT_STEAL_CENTER = ToRedLeft(NEXT_STEAL_CENTER);
                NEXT_STEAL_END = ToRedLeft(NEXT_STEAL_END);
                TROUGH_FIRST = ToRedLeft(TROUGH_FIRST);
                TROUGH_NEXT = ToRedLeft(TROUGH_NEXT);
                TROUGH_EA = ToRedLeft(TROUGH_EA);
                END_START = ToRedLeft(END_START);
                END_HUB = ToRedLeft(END_HUB);
                RAMP_FINISH = ToRedLeft(RAMP_FINISH);
                break;
            
            case "blue right":
                RAMP_START = ToBlueRight(RAMP_START);
                RAMP_CROSS = ToBlueRight(RAMP_CROSS);
                STEAL_START = ToBlueRight(STEAL_START);
                STEAL_START_EA = ToBlueRight(STEAL_START_EA);
                STEAL_CENTER = ToBlueRight(STEAL_CENTER);
                STEAL_CENTER_EA = ToBlueRight(STEAL_CENTER_EA);
                NEXT_STEAL_CENTER = ToBlueRight(NEXT_STEAL_CENTER);
                NEXT_STEAL_END = ToBlueRight(NEXT_STEAL_END);
                TROUGH_FIRST = ToBlueRight(TROUGH_FIRST);
                TROUGH_NEXT = ToBlueRight(TROUGH_NEXT);
                TROUGH_EA = ToBlueRight(TROUGH_EA);
                END_START = ToBlueRight(END_START);
                END_HUB = ToBlueRight(END_HUB);
                RAMP_FINISH = ToBlueRight(RAMP_FINISH);
                break;

            case "blue left":
                RAMP_START = ToBlueLeft(RAMP_START);
                RAMP_CROSS = ToBlueLeft(RAMP_CROSS);
                STEAL_START = ToBlueLeft(STEAL_START);
                STEAL_START_EA = ToBlueLeft(STEAL_START_EA);
                STEAL_CENTER = ToBlueLeft(STEAL_CENTER);
                STEAL_CENTER_EA = ToBlueLeft(STEAL_CENTER_EA);
                NEXT_STEAL_CENTER = ToBlueLeft(NEXT_STEAL_CENTER);
                NEXT_STEAL_END = ToBlueLeft(NEXT_STEAL_END);
                TROUGH_FIRST = ToBlueLeft(TROUGH_FIRST);
                TROUGH_NEXT = ToBlueLeft(TROUGH_NEXT);
                TROUGH_EA = ToBlueLeft(TROUGH_EA);
                END_START = ToBlueLeft(END_START);
                END_HUB = ToBlueLeft(END_HUB);
                RAMP_FINISH = ToBlueLeft(RAMP_FINISH);
                break;
        }

        setup();
    }

    private void setup() {
        addCommands(
            new InstantCommand(() -> {
                SubsystemManager.getDrivebase().resetOdometry(RAMP_START);
                SubsystemManager.getDrivebase().resetWaypointToPose();
            }),
            SubsystemManager.operateAuto(ShooterState.IDLE, IntakeState.INACTIVE, StorageState.IN),
            SubsystemManager.getDrivebase().driveToPose(
                new APTarget(RAMP_CROSS).withVelocity(1), 0.3, 10, true),
            SubsystemManager.operateAuto(ShooterState.IDLE, IntakeState.ACTIVE, StorageState.OUT),
            SubsystemManager.getDrivebase().waypointToPose(new APTarget[]{
                new APTarget(STEAL_START).withVelocity(3).withEntryAngle(STEAL_START_EA),
                new APTarget(STEAL_CENTER).withVelocity(1).withEntryAngle(STEAL_CENTER_EA),
                new APTarget(TROUGH_FIRST).withVelocity(0).withEntryAngle(TROUGH_EA)
            }, 0.3, 0.1, 5),
            SubsystemManager.operateAuto(ShooterState.DEPLETE, IntakeState.DEPLETE, StorageState.OUT),
            Commands.waitSeconds(4),
            SubsystemManager.operateAuto(ShooterState.IDLE, IntakeState.ACTIVE, StorageState.OUT),
            SubsystemManager.getDrivebase().waypointToPose(new APTarget[]{
                new APTarget(STEAL_START).withVelocity(2).withEntryAngle(STEAL_START_EA),
                new APTarget(NEXT_STEAL_CENTER).withVelocity(3).withEntryAngle(STEAL_CENTER_EA),
                new APTarget(NEXT_STEAL_END).withVelocity(2),
                new APTarget(TROUGH_NEXT).withVelocity(0).withEntryAngle(TROUGH_EA)
            }, 0.3, 0.1, 5),
            SubsystemManager.operateAuto(ShooterState.DEPLETE, IntakeState.DEPLETE, StorageState.OUT),
            Commands.waitSeconds(4),
            SubsystemManager.operateAuto(ShooterState.IDLE, IntakeState.ACTIVE, StorageState.OUT),
            SubsystemManager.getDrivebase().waypointToPose(new APTarget[]{
                new APTarget(END_START).withVelocity(3),
                new APTarget(END_HUB).withVelocity(3),
                new APTarget(RAMP_FINISH).withVelocity(0).withEntryAngle(TROUGH_EA)
            }, 0.3, false)
        );
    }
}