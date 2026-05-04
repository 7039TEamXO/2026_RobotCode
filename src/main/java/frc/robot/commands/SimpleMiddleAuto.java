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

public class SimpleMiddleAuto extends SequentialCommandGroup {
    private Pose2d RAMP_START;
    private Pose2d RAMP_CROSS;
    private Rotation2d RAMP_RETURN_EA; // Entry angle
    private Pose2d RAMP_CROSS_FINAL;
    private Pose2d RAMP_RETURN;

    private Pose2d RAMP_START_ALIGNED;

    private Pose2d STEAL_START;
    private Rotation2d STEAL_START_EA;
    private Pose2d STEAL_END;
    private Rotation2d STEAL_END_EA;

    private Pose2d RETURN_START;
    private Pose2d CLOSE_END;
    private Rotation2d CLOSE_END_EA;

    private double DELAY;

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

    // Chess, because in the beginning of the game there's usually a race to conquer the center, which is what's done here
    // Analogous to Short Shoot with the steal and return sequence flipped
    public SimpleMiddleAuto(String orientation, double delay) {
        // Default is "red right"
        RAMP_START = new Pose2d(13.2, 5.5, Rotation2d.k180deg);
        RAMP_CROSS = new Pose2d(10.9, 5.5, Rotation2d.k180deg);
        RAMP_RETURN_EA = Rotation2d.kZero;
        RAMP_START_ALIGNED = new Pose2d(13.2, 5.5, Rotation2d.fromDegrees(-131));
        RAMP_CROSS_FINAL = new Pose2d(10.3, 5.75, Rotation2d.fromDegrees(-90));
        RAMP_RETURN = new Pose2d(10.2, 5.5, Rotation2d.fromDegrees(-135));

        STEAL_START = new Pose2d(9, 4, Rotation2d.k180deg);
        STEAL_START_EA = Rotation2d.k180deg;
        STEAL_END = new Pose2d(8, 5.25, Rotation2d.kCCW_90deg);
        STEAL_END_EA = Rotation2d.kCCW_90deg;

        RETURN_START = new Pose2d(9, 6.5, Rotation2d.kZero);
        CLOSE_END = new Pose2d(10.5, 3, Rotation2d.kCW_90deg);
        CLOSE_END_EA = Rotation2d.kCW_90deg;

        DELAY = delay;
        
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
                RAMP_RETURN = ToRedLeft(RAMP_RETURN);
                RAMP_START_ALIGNED = ToRedLeft(RAMP_START_ALIGNED);
                STEAL_START = ToRedLeft(STEAL_START);
                STEAL_START_EA = ToRedLeft(STEAL_START_EA);
                STEAL_END = ToRedLeft(STEAL_END);
                STEAL_END_EA = ToRedLeft(STEAL_END_EA);
                RETURN_START = ToRedLeft(RETURN_START);
                CLOSE_END = ToRedLeft(CLOSE_END);
                CLOSE_END_EA = ToRedLeft(CLOSE_END_EA);
                break;
            
            case "blue right":
                RAMP_START = ToBlueRight(RAMP_START);
                RAMP_CROSS = ToBlueRight(RAMP_CROSS);
                RAMP_RETURN_EA = ToBlueRight(RAMP_RETURN_EA);
                RAMP_CROSS_FINAL = ToBlueRight(RAMP_CROSS_FINAL);
                RAMP_RETURN = ToBlueRight(RAMP_RETURN);
                RAMP_START_ALIGNED = ToBlueRight(RAMP_START_ALIGNED);
                STEAL_START = ToBlueRight(STEAL_START);
                STEAL_START_EA = ToBlueRight(STEAL_START_EA);
                STEAL_END = ToBlueRight(STEAL_END);
                STEAL_END_EA = ToBlueRight(STEAL_END_EA);
                RETURN_START = ToBlueRight(RETURN_START);
                CLOSE_END = ToBlueRight(CLOSE_END);
                CLOSE_END_EA = ToBlueRight(CLOSE_END_EA);
                break;

            case "blue left":
                RAMP_START = ToBlueLeft(RAMP_START);
                RAMP_CROSS = ToBlueLeft(RAMP_CROSS);
                RAMP_RETURN_EA = ToBlueLeft(RAMP_RETURN_EA);
                RAMP_CROSS_FINAL = ToBlueLeft(RAMP_CROSS_FINAL);
                RAMP_RETURN = ToBlueLeft(RAMP_RETURN);
                RAMP_START_ALIGNED = ToBlueLeft(RAMP_START_ALIGNED);
                STEAL_START = ToBlueLeft(STEAL_START);
                STEAL_START_EA = ToBlueLeft(STEAL_START_EA);
                STEAL_END = ToBlueLeft(STEAL_END);
                STEAL_END_EA = ToBlueLeft(STEAL_END_EA);
                RETURN_START = ToBlueLeft(RETURN_START);
                CLOSE_END = ToBlueLeft(CLOSE_END);
                CLOSE_END_EA = ToBlueLeft(CLOSE_END_EA);
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
                new APTarget(RAMP_CROSS).withVelocity(DELAY == 0 ? 1 : 0), 0.3, 10, false),
                SubsystemManager.operateAuto(ShooterState.IDLE, IntakeState.ACTIVE, StorageState.OUT),
                Commands.waitSeconds(DELAY),
            SubsystemManager.getDrivebase().waypointToPose(new APTarget[]{
                new APTarget(STEAL_START).withVelocity(2).withEntryAngle(STEAL_START_EA),
                new APTarget(STEAL_END).withVelocity(1).withEntryAngle(STEAL_END_EA),
                new APTarget(RETURN_START).withVelocity(1).withEntryAngle(RAMP_RETURN_EA),
                new APTarget(RAMP_RETURN).withVelocity(1)
            }, 0.3, 0.3, 30),
            SubsystemManager.operateAuto(ShooterState.SPEED_UP, IntakeState.ACTIVE, StorageState.OUT),
            SubsystemManager.getDrivebase().driveToPose(
                new APTarget(RAMP_START_ALIGNED).withVelocity(0), 0.05, 2, false),
            SubsystemManager.operateAuto(ShooterState.SHOOT, IntakeState.HELP_SHOOT, StorageState.HELP_SHOOT),
            Commands.waitSeconds(4),
            SubsystemManager.operateAuto(ShooterState.IDLE, IntakeState.ACTIVE, StorageState.OUT),
            SubsystemManager.getDrivebase().driveToPose(
                new APTarget(RAMP_CROSS_FINAL).withVelocity(0.5), 0.3, 30, true),
            SubsystemManager.operateAuto(ShooterState.IDLE, IntakeState.ACTIVE, StorageState.OUT),
            SubsystemManager.getDrivebase().waypointToPose(new APTarget[]{
                new APTarget(CLOSE_END).withVelocity(1).withEntryAngle(CLOSE_END_EA),
                new APTarget(RAMP_RETURN).withVelocity(1)
            }, 0.3, 0.3, 30),
            SubsystemManager.operateAuto(ShooterState.SPEED_UP, IntakeState.ACTIVE, StorageState.OUT),
            SubsystemManager.getDrivebase().driveToPose(
                new APTarget(RAMP_START_ALIGNED).withVelocity(0), 0.03, 2, false),
            SubsystemManager.operateAuto(ShooterState.SHOOT, IntakeState.HELP_SHOOT, StorageState.HELP_SHOOT),
            Commands.waitSeconds(3),
            SubsystemManager.operateAuto(ShooterState.IDLE, IntakeState.INACTIVE, StorageState.IN),
            SubsystemManager.getDrivebase().driveToPose(
                new APTarget(RAMP_CROSS_FINAL).withVelocity(0.6), 0.3, 30, true)
        );
    }
}
