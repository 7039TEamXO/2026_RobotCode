package frc.robot.commands;

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

public class CenterAuto extends SequentialCommandGroup {
    private Pose2d START;

    private Rotation2d ToBlueLeft(Rotation2d angle) {
        return angle.plus(Rotation2d.k180deg).times(-1);
    }

    private Pose2d ToBlueLeft(Pose2d pose) {
        return new Pose2d(Constants.Field.FIELD_WIDTH - pose.getX(), pose.getY(), ToBlueLeft(pose.getRotation()));
    }

    public CenterAuto(String orientation) {
        // Default is "red right"
        START = new Pose2d(13.3, Constants.Field.FIELD_HEIGHT / 2, Rotation2d.k180deg);

        switch(orientation) {
            default:
            case "red left":
            case "red right":
                // Do nothing
                break;
            
            case "blue left":
            case "blue right":
                START = ToBlueLeft(START);
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
            SubsystemManager.operateAuto(ShooterState.SPEED_UP, IntakeState.INACTIVE, StorageState.IN),
            Commands.waitSeconds(1),
            SubsystemManager.operateAuto(ShooterState.SHOOT, IntakeState.HELP_SHOOT, StorageState.IN),
            Commands.waitSeconds(7),
            SubsystemManager.operateAuto(ShooterState.SHOOT, IntakeState.INACTIVE, StorageState.IN),
            Commands.waitSeconds(4),
            SubsystemManager.operateAuto(ShooterState.IDLE, IntakeState.INACTIVE, StorageState.IN)
        );
    }
}
