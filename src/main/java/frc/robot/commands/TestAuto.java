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
                    SubsystemManager.getDrivebase().resetOdometry(new Pose2d(0, 0, Rotation2d.kZero));
                SubsystemManager.getDrivebase().resetWaypointToPose();
            }),
            SubsystemManager.getDrivebase().waypointToPose(new APTarget[]{
                new APTarget(new Pose2d(4, 0, Rotation2d.kZero)).withEntryAngle(Rotation2d.kZero).withVelocity(0)
            })
        );
    }
}