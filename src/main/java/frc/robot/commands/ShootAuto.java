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
    private Pose2d RAMP_CROSS_FINAL;
    private Pose2d RAMP_START_FINAL;

    private Pose2d RAMP_START_ALIGNED;

    private Pose2d STEAL_START;
    private Rotation2d STEAL_START_EA;
    private Pose2d STEAL_END;
    private Rotation2d STEAL_END_EA;
    private Pose2d STEAL_FURTHER_END;

    private Pose2d CLOSE_ZERO;
    private Pose2d CLOSE_START;
    private Rotation2d CLOSE_START_EA;
    private Pose2d CLOSE_HUB;

    private Pose2d GO_BACK_OPPOSITE_SIDE;
    private Pose2d RAMP_CROSS_FINAL_OTHER_SIDE;
    private Pose2d RAMP_CROSS_START_OTHER_SIDE;
    private Pose2d COLLECT_CLOSE_OPPOSITE_SIDE_START;
    private Pose2d COLLECT_CLOSE_OPPOSITE_SIDE_FINAL;


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
        RAMP_START = new Pose2d(13.2, 5.5, Rotation2d.k180deg);
        RAMP_CROSS = new Pose2d(10.8, 5.7, Rotation2d.k180deg);
        RAMP_RETURN_EA = Rotation2d.kZero;
        RAMP_START_ALIGNED = new Pose2d(13.2, 5.5, Rotation2d.fromDegrees(-131));
        RAMP_CROSS_FINAL = new Pose2d(10.8, 5.3, Rotation2d.k180deg);
        RAMP_START_FINAL = new Pose2d(13.2, 5.5, Rotation2d.k180deg);

        STEAL_START = new Pose2d(8, 6, Rotation2d.fromDegrees(225));
        STEAL_START_EA = Rotation2d.fromDegrees(225);
        STEAL_END = new Pose2d(8, 2.5, Rotation2d.kCW_90deg);
        STEAL_END_EA = Rotation2d.kCW_90deg;
        STEAL_FURTHER_END = new Pose2d(9, 2.5, Rotation2d.kCW_90deg);

        CLOSE_ZERO = new Pose2d(9.75, 2.5, Rotation2d.kZero);
        CLOSE_START = new Pose2d(10.5, 2.5, Rotation2d.kCCW_90deg);
        CLOSE_START_EA = Rotation2d.kCCW_90deg;
        CLOSE_HUB = new Pose2d(10.5, 2.5, Rotation2d.kCCW_90deg);

        GO_BACK_OPPOSITE_SIDE = new Pose2d(13.2, 2.5, Rotation2d.fromDegrees(131));
        RAMP_CROSS_FINAL_OTHER_SIDE = new Pose2d(10.5, 2.5, Rotation2d.k180deg);
        RAMP_CROSS_START_OTHER_SIDE = new Pose2d(13.2, 2.5, Rotation2d.k180deg);

        COLLECT_CLOSE_OPPOSITE_SIDE_START = new Pose2d(8.5, 2.5, Rotation2d.kCCW_90deg);
        COLLECT_CLOSE_OPPOSITE_SIDE_FINAL = new Pose2d(8.5, 5, Rotation2d.kCCW_90deg);


        switch(orientation) {
            default:
            case "red right":
                // Do nothing
                break;
            
            case "red left":
                RAMP_START = ToRedLeft(RAMP_START);
                RAMP_CROSS = ToRedLeft(RAMP_CROSS);
                RAMP_RETURN_EA = ToRedLeft(RAMP_RETURN_EA);
                RAMP_CROSS_FINAL = ToRedLeft(RAMP_CROSS_FINAL);
                RAMP_START_FINAL = ToRedLeft(RAMP_START_FINAL);
                RAMP_START_ALIGNED = ToRedLeft(RAMP_START_ALIGNED);
                GO_BACK_OPPOSITE_SIDE = ToRedLeft(GO_BACK_OPPOSITE_SIDE);
                RAMP_CROSS_FINAL_OTHER_SIDE = ToRedLeft(RAMP_CROSS_FINAL_OTHER_SIDE);
                RAMP_CROSS_START_OTHER_SIDE = ToRedLeft(RAMP_CROSS_START_OTHER_SIDE);
                STEAL_START = ToRedLeft(STEAL_START);
                STEAL_START_EA = ToRedLeft(STEAL_START_EA);
                STEAL_END = ToRedLeft(STEAL_END);
                STEAL_END_EA = ToRedLeft(STEAL_END_EA);
                COLLECT_CLOSE_OPPOSITE_SIDE_START = ToRedLeft(COLLECT_CLOSE_OPPOSITE_SIDE_START);
                COLLECT_CLOSE_OPPOSITE_SIDE_FINAL = ToRedLeft(COLLECT_CLOSE_OPPOSITE_SIDE_FINAL);
                STEAL_FURTHER_END = ToRedLeft(STEAL_FURTHER_END);
                CLOSE_ZERO = ToRedLeft(CLOSE_ZERO);
                CLOSE_START = ToRedLeft(CLOSE_START);
                CLOSE_START_EA = ToRedLeft(CLOSE_START_EA);
                CLOSE_HUB = ToRedLeft(CLOSE_HUB);
                break;
            
            case "blue right":
                RAMP_START = ToBlueRight(RAMP_START);
                RAMP_CROSS = ToBlueRight(RAMP_CROSS);
                RAMP_RETURN_EA = ToBlueRight(RAMP_RETURN_EA);
                RAMP_CROSS_FINAL = ToBlueRight(RAMP_CROSS_FINAL);
                RAMP_START_FINAL = ToBlueRight(RAMP_START_FINAL);
                RAMP_START_ALIGNED = ToBlueRight(RAMP_START_ALIGNED);
                STEAL_START = ToBlueRight(STEAL_START);
                STEAL_START_EA = ToBlueRight(STEAL_START_EA);
                STEAL_END = ToBlueRight(STEAL_END);
                STEAL_END_EA = ToBlueRight(STEAL_END_EA);
                STEAL_FURTHER_END = ToBlueRight(STEAL_FURTHER_END);
                CLOSE_ZERO = ToBlueRight(CLOSE_ZERO);
                CLOSE_START = ToBlueRight(CLOSE_START);
                CLOSE_START_EA = ToBlueRight(CLOSE_START_EA);
                CLOSE_HUB = ToBlueRight(CLOSE_HUB);
                break;

            case "blue left":
                RAMP_START = ToBlueLeft(RAMP_START);
                RAMP_CROSS = ToBlueLeft(RAMP_CROSS);
                RAMP_RETURN_EA = ToBlueLeft(RAMP_RETURN_EA);
                RAMP_CROSS_FINAL = ToBlueLeft(RAMP_CROSS_FINAL);
                RAMP_START_FINAL = ToBlueLeft(RAMP_START_FINAL);
                RAMP_START_ALIGNED = ToBlueLeft(RAMP_START_ALIGNED);
                STEAL_START = ToBlueLeft(STEAL_START);
                STEAL_START_EA = ToBlueLeft(STEAL_START_EA);
                STEAL_END = ToBlueLeft(STEAL_END);
                STEAL_END_EA = ToBlueLeft(STEAL_END_EA);
                STEAL_FURTHER_END = ToBlueLeft(STEAL_FURTHER_END);
                CLOSE_ZERO = ToBlueLeft(CLOSE_ZERO);
                CLOSE_START = ToBlueLeft(CLOSE_START);
                CLOSE_START_EA = ToBlueLeft(CLOSE_START_EA);
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
                new APTarget(RAMP_CROSS).withVelocity(1), 0.3, 10, true),
            SubsystemManager.operateAuto(ShooterState.IDLE, IntakeState.ACTIVE, StorageState.OUT),
            SubsystemManager.getDrivebase().waypointToPose(new APTarget[]{
                new APTarget(STEAL_START).withVelocity(3).withEntryAngle(STEAL_START_EA),
                new APTarget(STEAL_END).withVelocity(1).withEntryAngle(STEAL_END_EA),
                // new APTarget(CLOSE_ZERO).withVelocity(1).withEntryAngle(RAMP_RETURN_EA),
                // new APTarget(CLOSE_START).withVelocity(2).withEntryAngle(CLOSE_START_EA),
                // new APTarget(CLOSE_HUB).withVelocity(1),
                new APTarget(RAMP_CROSS_FINAL_OTHER_SIDE).withVelocity(1).withEntryAngle(RAMP_RETURN_EA)
            }, 0.3, 0.3, 30),
            SubsystemManager.getDrivebase().driveToPose(
                new APTarget(RAMP_CROSS_START_OTHER_SIDE).withVelocity(0), true, true),
            SubsystemManager.operateAuto(ShooterState.SPEED_UP, IntakeState.INACTIVE, StorageState.IN),
            SubsystemManager.getDrivebase().driveToPose(new APTarget(GO_BACK_OPPOSITE_SIDE), 0.03, 2, false),
            SubsystemManager.operateAuto(ShooterState.SHOOT, IntakeState.HELP_SHOOT, StorageState.HELP_SHOOT),
            Commands.waitSeconds(8),
            SubsystemManager.operateAuto(ShooterState.IDLE, IntakeState.ACTIVE, StorageState.IN),
            SubsystemManager.getDrivebase().driveToPose(
                new APTarget(RAMP_CROSS_FINAL_OTHER_SIDE).withVelocity(1), true, true),
            SubsystemManager.operateAuto(ShooterState.IDLE, IntakeState.ACTIVE, StorageState.OUT),   
            SubsystemManager.getDrivebase().waypointToPose(new APTarget[]{
                new APTarget(COLLECT_CLOSE_OPPOSITE_SIDE_START).withVelocity(3),
                new APTarget(COLLECT_CLOSE_OPPOSITE_SIDE_FINAL).withVelocity(0)
            }, 0.3, 0.1, 10)
        );
    }
}