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
                    SubsystemManager.getDrivebase().resetOdometry(new Pose2d(7, 4, new Rotation2d()));
                SubsystemManager.getDrivebase().resetWaypointToPose();
            }),
            SubsystemManager.getDrivebase().waypointToPose(new APTarget[]{
                new APTarget(new Pose2d(7, 7, new Rotation2d())).withVelocity(4).withEntryAngle(new Rotation2d()), new APTarget(new Pose2d(10, 7, new Rotation2d())).withVelocity(3).withEntryAngle(Rotation2d.fromDegrees(0)), new APTarget(new Pose2d(12.25, 5.1, Rotation2d.fromDegrees(-60)))
            })
        );
    }
}