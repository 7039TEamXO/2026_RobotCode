package frc.robot.commands;

import com.therekrab.autopilot.APTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.Limelight;
import frc.robot.subsystems.SubsystemManager;
public class TestAuto extends SequentialCommandGroup {
    public TestAuto() {
        addCommands(
            new InstantCommand(() -> {
                if(!Limelight.hasTarget() || Robot.isSimulation()) 
                    SubsystemManager.getDrivebase().resetOdometry(new Pose2d(13, 3.5, Rotation2d.kZero));
                SubsystemManager.getDrivebase().resetWaypointToPose();
            }),
            SubsystemManager.getDrivebase().waypointToPose(new APTarget[]{
                new APTarget(new Pose2d(13.2, 5.2, Rotation2d.fromDegrees(45))).withVelocity(2).withEntryAngle(Rotation2d.k180deg),
                new APTarget(new Pose2d(10.6, 5.9, Rotation2d.fromDegrees(45))).withVelocity(2),
                new APTarget(new Pose2d(8.725, 6.8, Rotation2d.kCW_90deg)).withVelocity(0).withEntryAngle(Rotation2d.k180deg),
                new APTarget(new Pose2d(8.725, 5, Rotation2d.kCW_90deg)).withVelocity(0)
            }, 0.3).until(() -> SubsystemManager.getDrivebase().isCloseEnoughToPose(new Pose2d(8.725, 5, Rotation2d.kCW_90deg))),
            new InstantCommand(() -> {
                SubsystemManager.getDrivebase().resetWaypointToPose();
            }),
            SubsystemManager.getDrivebase().waypointToPose(new APTarget[]{
                new APTarget(new Pose2d(8.725, 5, Rotation2d.fromDegrees(20))).withVelocity(0)
            }, 0.1).withTimeout(2),
            SubsystemManager.getDrivebase().waypointToPose(new APTarget[]{
                new APTarget(new Pose2d(10.6, 5.9, Rotation2d.fromDegrees(45))).withVelocity(2).withEntryAngle(Rotation2d.kZero),
                new APTarget(new Pose2d(13.2, 5.2, Rotation2d.fromDegrees(45))).withVelocity(2),
                new APTarget(new Pose2d(13, 3.5, Rotation2d.kZero)).withEntryAngle(Rotation2d.k180deg).withVelocity(0)
            }, 0.2)
        );
    }
}