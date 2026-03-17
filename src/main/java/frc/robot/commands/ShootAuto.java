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

public class ShootAuto extends SequentialCommandGroup {
    private Pose2d RAMP_START;
    private Pose2d RAMP_CROSS;
    private Rotation2d RAMP_RETURN_EA; // Entry angle

    private Pose2d RAMP_START_ALIGNED;

    private Pose2d STEAL_START;
    private Rotation2d STEAL_START_EA;
    private Pose2d STEAL_END;
    private Rotation2d STEAL_END_EA;

    private Pose2d CLOSE_START;
    private Pose2d CLOSE_HUB;

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

    public ShootAuto(String orientation) {
        // Default is "red right"
        RAMP_START = new Pose2d(13.2, 5.4, Rotation2d.kCW_90deg);
        RAMP_CROSS = new Pose2d(10.8, 5.8, Rotation2d.kCW_90deg);
        RAMP_RETURN_EA = Rotation2d.kZero;
        RAMP_START_ALIGNED = new Pose2d(13.2, 5.4, Rotation2d.fromDegrees(-132));
        STEAL_START = new Pose2d(9.5, 7, Rotation2d.kCW_90deg);
        STEAL_START_EA = Rotation2d.k180deg;
        STEAL_END = new Pose2d(9, 4, Rotation2d.kCW_90deg);
        STEAL_END_EA = Rotation2d.fromDegrees(-45);
        CLOSE_START = new Pose2d(9.5, 6.5, Rotation2d.kCW_90deg);
        CLOSE_HUB = new Pose2d(10.8, 3, Rotation2d.kCW_90deg);

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
                CLOSE_START = ToRedLeft(CLOSE_START);
                CLOSE_HUB = ToRedLeft(CLOSE_HUB);
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
                CLOSE_START = ToBlueRight(CLOSE_START);
                CLOSE_HUB = ToBlueRight(CLOSE_HUB);
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
                CLOSE_START = ToBlueLeft(CLOSE_START);
                CLOSE_HUB = ToBlueLeft(CLOSE_HUB);
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
                new APTarget(RAMP_CROSS).withVelocity(3), true),
            SubsystemManager.operateAuto(ShooterState.IDLE, IntakeState.ACTIVE, StorageState.OUT),
            SubsystemManager.getDrivebase().waypointToPose(new APTarget[]{
                new APTarget(STEAL_START).withVelocity(3).withEntryAngle(STEAL_START_EA),
                new APTarget(STEAL_END).withVelocity(1).withEntryAngle(STEAL_END_EA),
                new APTarget(RAMP_CROSS).withVelocity(3).withEntryAngle(RAMP_RETURN_EA)
            }, 0.3, true),
            SubsystemManager.operateAuto(ShooterState.SPEED_UP, IntakeState.INACTIVE, StorageState.OUT),
            SubsystemManager.getDrivebase().driveToPose(new APTarget(RAMP_START_ALIGNED), 0.03, 2),
            SubsystemManager.operateAuto(ShooterState.SHOOT, IntakeState.HELP_SHOOT, StorageState.HELP_SHOOT),
            Commands.waitSeconds(5),
            SubsystemManager.operateAuto(ShooterState.IDLE, IntakeState.ACTIVE, StorageState.OUT),
            SubsystemManager.getDrivebase().waypointToPose(new APTarget[]{
                new APTarget(RAMP_CROSS).withVelocity(3),
                new APTarget(CLOSE_START).withVelocity(2).withEntryAngle(STEAL_START_EA),
                new APTarget(CLOSE_HUB).withVelocity(1).withEntryAngle(STEAL_END_EA),
                new APTarget(RAMP_CROSS).withVelocity(3).withEntryAngle(RAMP_RETURN_EA)
            }, 0.3, true),
            SubsystemManager.operateAuto(ShooterState.SPEED_UP, IntakeState.INACTIVE, StorageState.OUT),
            SubsystemManager.getDrivebase().driveToPose(new APTarget(RAMP_START_ALIGNED), 0.03, 2),
            SubsystemManager.operateAuto(ShooterState.SHOOT, IntakeState.HELP_SHOOT, StorageState.HELP_SHOOT)
        );
    }
}