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

public class DelayShortCollectAuto extends SequentialCommandGroup {
    private Pose2d RAMP_START;
    private Pose2d RAMP_CROSS;
    private Rotation2d RAMP_RETURN_EA; // Entry angle
    private Pose2d RAMP_START_ALIGNED;

    private Pose2d STEAL_START;
    private Rotation2d STEAL_START_EA;
    private Pose2d STEAL_END;
    private Rotation2d STEAL_END_EA;

    private Pose2d RETURN_START;
    private Pose2d RETURN_END;

    private Pose2d TROUGH;

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

    public DelayShortCollectAuto(String orientation) {
        // Default is "red right"
        RAMP_START = new Pose2d(13.2, 5.5, Rotation2d.k180deg);
        RAMP_CROSS = new Pose2d(10.6, 5.7, Rotation2d.k180deg);
        RAMP_RETURN_EA = Rotation2d.kZero;
        RAMP_START_ALIGNED = new Pose2d(13.2, 5.5, Rotation2d.fromDegrees(-131));

        STEAL_START = new Pose2d(8.5, 6.5, Rotation2d.k180deg);
        STEAL_START_EA = Rotation2d.k180deg;
        STEAL_END = new Pose2d(6.5, 5.5, Rotation2d.kCW_90deg);
        STEAL_END_EA = Rotation2d.kCW_90deg;

        RETURN_START = new Pose2d(8.5, 4.75, Rotation2d.kZero);
        RETURN_END = new Pose2d(9.75, 5.75, Rotation2d.fromDegrees(45));
        TROUGH = new Pose2d(11, 7.4, Rotation2d.kZero);

        switch(orientation) {
            default:
            case "red right":
                // Do nothing
                break;
            
            case "red left":
                RAMP_START = ToRedLeft(RAMP_START);
                RAMP_CROSS = ToRedLeft(RAMP_CROSS);
                RAMP_RETURN_EA = ToRedLeft(RAMP_RETURN_EA);
                RAMP_START_ALIGNED = ToRedLeft(RAMP_START_ALIGNED);
                STEAL_START = ToRedLeft(STEAL_START);
                STEAL_START_EA = ToRedLeft(STEAL_START_EA);
                STEAL_END = ToRedLeft(STEAL_END);
                STEAL_END_EA = ToRedLeft(STEAL_END_EA);
                RETURN_START = ToRedLeft(RETURN_START);
                RETURN_END = ToRedLeft(RETURN_END);
                TROUGH = ToRedLeft(TROUGH);
                break;
            
            case "blue right":
                RAMP_START = ToBlueRight(RAMP_START);
                RAMP_CROSS = ToBlueRight(RAMP_CROSS);
                RAMP_RETURN_EA = ToBlueRight(RAMP_RETURN_EA);
                RAMP_START_ALIGNED = ToBlueRight(RAMP_START_ALIGNED);
                STEAL_START = ToBlueRight(STEAL_START);
                STEAL_START_EA = ToBlueRight(STEAL_START_EA);
                STEAL_END = ToBlueRight(STEAL_END);
                STEAL_END_EA = ToBlueRight(STEAL_END_EA);
                RETURN_START = ToBlueRight(RETURN_START);
                RETURN_END = ToBlueRight(RETURN_END);
                TROUGH = ToBlueRight(TROUGH);
                break;

            case "blue left":
                RAMP_START = ToBlueLeft(RAMP_START);
                RAMP_CROSS = ToBlueLeft(RAMP_CROSS);
                RAMP_RETURN_EA = ToBlueLeft(RAMP_RETURN_EA);
                RAMP_START_ALIGNED = ToBlueLeft(RAMP_START_ALIGNED);
                STEAL_START = ToBlueLeft(STEAL_START);
                STEAL_START_EA = ToBlueLeft(STEAL_START_EA);
                STEAL_END = ToBlueLeft(STEAL_END);
                STEAL_END_EA = ToBlueLeft(STEAL_END_EA);
                RETURN_START = ToBlueLeft(RETURN_START);
                RETURN_END = ToBlueLeft(RETURN_END);
                TROUGH = ToBlueLeft(TROUGH);
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
            SubsystemManager.operateAuto(ShooterState.SPEED_UP, IntakeState.INACTIVE, StorageState.IN),
            SubsystemManager.getDrivebase().driveToPose(
                new APTarget(RAMP_START_ALIGNED).withVelocity(0), 0.03, 2, true),
            SubsystemManager.operateAuto(ShooterState.SHOOT, IntakeState.HELP_SHOOT, StorageState.IN),
            Commands.waitSeconds(3),
            SubsystemManager.operateAuto(ShooterState.IDLE, IntakeState.INACTIVE, StorageState.IN),
            SubsystemManager.getDrivebase().driveToPose(
                new APTarget(RAMP_CROSS).withVelocity(1), 0.3, 10, true),
            SubsystemManager.operateAuto(ShooterState.IDLE, IntakeState.ACTIVE, StorageState.OUT),
            SubsystemManager.getDrivebase().waypointToPose(new APTarget[]{
                new APTarget(STEAL_START).withVelocity(3).withEntryAngle(STEAL_START_EA),
                new APTarget(STEAL_END).withVelocity(1).withEntryAngle(STEAL_END_EA),
                new APTarget(RETURN_START).withVelocity(1).withEntryAngle(RAMP_RETURN_EA),
                new APTarget(RETURN_END).withVelocity(2),
                new APTarget(TROUGH).withVelocity(0).withEntryAngle(RAMP_RETURN_EA)
            }, 0.3, 0.1, 10),
            SubsystemManager.operateAuto(ShooterState.SHOOT, IntakeState.DEPLETE, StorageState.OUT),
            Commands.waitSeconds(4),
            SubsystemManager.operateAuto(ShooterState.IDLE, IntakeState.ACTIVE, StorageState.OUT),
            SubsystemManager.getDrivebase().waypointToPose(new APTarget[]{
                new APTarget(STEAL_START).withVelocity(3).withEntryAngle(STEAL_START_EA),
                new APTarget(STEAL_END).withVelocity(1).withEntryAngle(STEAL_END_EA),
                new APTarget(RETURN_START).withVelocity(1).withEntryAngle(RAMP_RETURN_EA),
                new APTarget(RETURN_END).withVelocity(2),
                new APTarget(TROUGH).withVelocity(0).withEntryAngle(RAMP_RETURN_EA)
            }, 0.3, 0.1, 10),
            SubsystemManager.operateAuto(ShooterState.SHOOT, IntakeState.DEPLETE, StorageState.OUT),
            Commands.waitSeconds(4),
            SubsystemManager.operateAuto(ShooterState.IDLE, IntakeState.INACTIVE, StorageState.OUT),
            SubsystemManager.getDrivebase().driveToPose(
                new APTarget(RAMP_CROSS).withVelocity(1).withEntryAngle(RAMP_RETURN_EA), false, false)
        );
    }
}