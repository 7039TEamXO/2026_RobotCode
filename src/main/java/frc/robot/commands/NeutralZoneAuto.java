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
import frc.robot.subsystems.SwerveDrive.SwerveSubsystem;

// Basic autonomy for taking out  balls from Neutral Zone and shooting them to the Hub. Endpoint at shooting position (between hub and ramp)

public class NeutralZoneAuto extends SequentialCommandGroup {
    private Pose2d START;
    private Pose2d RAMP_ENTRANCE;
    private Rotation2d RAMP_ENTRANCE_EA; // EA = Entry Angle
    private Pose2d RAMP_EXIT;

    private Pose2d COLLECT_START;
    private Rotation2d COLLECT_START_EA;
    private Pose2d HUB_SHOOT_POS;
    private Pose2d COLLECT_END;


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

    public NeutralZoneAuto(String orientation) {
        // default is red right
        START = new Pose2d(13, 3.5, Rotation2d.kZero);
        RAMP_ENTRANCE = new Pose2d(13.2, 5.5, Rotation2d.fromDegrees(-45));
        RAMP_ENTRANCE_EA = Rotation2d.k180deg;
        RAMP_EXIT = new Pose2d(10.6, 5.5, Rotation2d.fromDegrees(-45));
        COLLECT_START = new Pose2d(8.725, 6.8, Rotation2d.kCW_90deg);
        COLLECT_START_EA = Rotation2d.k180deg;
        HUB_SHOOT_POS = new Pose2d(13.3, 5.2,  Rotation2d.fromDegrees(135));
        COLLECT_END = new Pose2d(8.725, 2.3, Rotation2d.kCW_90deg);

        switch(orientation) {
            default:
            case "red right":
                // Do nothing, because we already have their points
                break;
            
            case "red left":
                START = ToRedLeft(START);
                RAMP_ENTRANCE = ToRedLeft(RAMP_ENTRANCE);
                RAMP_ENTRANCE_EA = ToRedLeft(RAMP_ENTRANCE_EA);
                RAMP_EXIT = ToRedLeft(RAMP_EXIT);
                COLLECT_START = ToRedLeft(COLLECT_START);
                COLLECT_START_EA = ToRedLeft(COLLECT_START_EA);
                HUB_SHOOT_POS = ToRedLeft(HUB_SHOOT_POS);
                COLLECT_END = ToRedLeft(COLLECT_END);
                break;
            
            case "blue right":
                START = ToBlueRight(START);
                RAMP_ENTRANCE = ToBlueRight(RAMP_ENTRANCE);
                RAMP_ENTRANCE_EA = ToBlueRight(RAMP_ENTRANCE_EA);
                RAMP_EXIT = ToBlueRight(RAMP_EXIT);
                COLLECT_START = ToBlueRight(COLLECT_START);
                COLLECT_START_EA = ToBlueRight(COLLECT_START_EA);
                HUB_SHOOT_POS = ToBlueRight(HUB_SHOOT_POS);
                COLLECT_END = ToBlueRight(COLLECT_END);
                break;

            case "blue left":
                START = ToBlueLeft(START);
                RAMP_ENTRANCE = ToBlueLeft(RAMP_ENTRANCE);
                RAMP_ENTRANCE_EA = ToBlueLeft(RAMP_ENTRANCE_EA);
                RAMP_EXIT = ToBlueLeft(RAMP_EXIT);
                COLLECT_START = ToBlueLeft(COLLECT_START);
                COLLECT_START_EA = ToBlueLeft(COLLECT_START_EA);
                HUB_SHOOT_POS = ToBlueLeft(HUB_SHOOT_POS);
                COLLECT_END = ToBlueLeft(COLLECT_END);
                break;
        }

        setup();
    }

    private void setup() {
        addCommands(
            new InstantCommand(() -> {
                SubsystemManager.getDrivebase().resetOdometry(START);
                SubsystemManager.getDrivebase().resetWaypointToPose();
            }),
            SubsystemManager.operateAuto(ShooterState.IDLE, IntakeState.INACTIVE, StorageState.IN),
            SubsystemManager.getDrivebase().waypointToPose(new APTarget[]{
                new APTarget(RAMP_ENTRANCE).withVelocity(1).withEntryAngle(RAMP_ENTRANCE_EA),
                new APTarget(RAMP_EXIT).withVelocity(1)
            }, 0.3, true),
            SubsystemManager.operateAuto(ShooterState.IDLE, IntakeState.ACTIVE, StorageState.OUT),
            SubsystemManager.getDrivebase().waypointToPose(new APTarget[]{
                new APTarget(COLLECT_START).withVelocity(0).withEntryAngle(COLLECT_START_EA),
                new APTarget(COLLECT_END)
            }, 0.3, true),
            SubsystemManager.getDrivebase().waypointToPose(new APTarget[]{
                new APTarget(COLLECT_START).withVelocity(0).withEntryAngle(Rotation2d.k180deg),
                new APTarget(RAMP_EXIT).withVelocity(0).withEntryAngle(RAMP_ENTRANCE_EA),
                new APTarget(RAMP_ENTRANCE).withVelocity(0),
            }, 0, null),
            SubsystemManager.operateAuto(ShooterState.SPEED_UP, IntakeState.ACTIVE, StorageState.OUT),
            SubsystemManager.getDrivebase().driveToPose(new APTarget(HUB_SHOOT_POS).withVelocity(0), 0.1, 5),
            SubsystemManager.operateAuto(ShooterState.SHOOT, IntakeState.INACTIVE, StorageState.OUT),
            Commands.waitSeconds(3),
            SubsystemManager.operateAuto(ShooterState.IDLE, IntakeState.INACTIVE, StorageState.IN)
        );
    }
}

