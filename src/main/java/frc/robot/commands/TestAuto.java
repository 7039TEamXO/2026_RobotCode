package frc.robot.commands;

import com.therekrab.autopilot.APTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.Limelight;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.Delivery.Shooter.ShooterState;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Intake.StorageState;
import frc.robot.subsystems.SwerveDrive.SwerveSubsystem;
public class TestAuto extends SequentialCommandGroup {
    public final Pose2d START;
    public final Pose2d RAMP_ENTRANCE;
    public final Pose2d RAMP_EXIT;
    public final Pose2d STEAL_START;
    public final Pose2d SHOOT_FIRST_ENTRANCE;
    public final Pose2d SHOOT_FIRST;
    public final Pose2d STEAL_END;
    public final Pose2d RAMP_NEXT_ENTRANCE;
    public final Pose2d RAMP_NEXT_EXIT;
    public final Pose2d FINAL;

    public TestAuto() {
        START = new Pose2d(13, 3.5, Rotation2d.kZero);
        RAMP_ENTRANCE = new Pose2d(13.2, 5.2, Rotation2d.fromDegrees(-45));
        RAMP_EXIT = new Pose2d(10.6, 5.9, Rotation2d.fromDegrees(-45));
        STEAL_START = new Pose2d(8.725, 6.8, Rotation2d.kCW_90deg);
        SHOOT_FIRST_ENTRANCE = new Pose2d(8.725, 4.5, Rotation2d.kCW_90deg);
        SHOOT_FIRST = new Pose2d(8.725, 4.5, Rotation2d.fromDegrees(30));
        STEAL_END = new Pose2d(8.725, 2.3, Rotation2d.kCW_90deg);
        RAMP_NEXT_ENTRANCE = new Pose2d(10.6, 2.3, Rotation2d.fromDegrees(-45));
        RAMP_NEXT_EXIT = new Pose2d(13.2, 2.8, Rotation2d.fromDegrees(-45));
        FINAL = SwerveSubsystem.getTowerPose();

        setup();
    }

    private void setup() {
        addCommands(
            new InstantCommand(() -> {
                if(!Limelight.hasTarget() || Robot.isSimulation()) 
                    SubsystemManager.getDrivebase().resetOdometry(START);
            }),
            SubsystemManager.operateAuto(ShooterState.IDLE, IntakeState.INACTIVE, StorageState.IN),
            SubsystemManager.getDrivebase().waypointToPose(new APTarget[]{
                new APTarget(RAMP_ENTRANCE).withVelocity(1).withEntryAngle(Rotation2d.k180deg),
                new APTarget(RAMP_EXIT).withVelocity(2)
            }, 0.3),
            SubsystemManager.operateAuto(ShooterState.IDLE, IntakeState.ACTIVE, StorageState.OUT),
            SubsystemManager.getDrivebase().waypointToPose(new APTarget[]{
                new APTarget(STEAL_START).withVelocity(0).withEntryAngle(Rotation2d.k180deg),
                new APTarget(SHOOT_FIRST_ENTRANCE).withVelocity(0)
            }, 0.2),
            SubsystemManager.operateAuto(ShooterState.SPEED_UP, IntakeState.ACTIVE, StorageState.OUT),
            SubsystemManager.getDrivebase().driveToPose(new APTarget(SHOOT_FIRST).withVelocity(0), 0.1, 1),
            SubsystemManager.operateAuto(ShooterState.SHOOT, IntakeState.INACTIVE, StorageState.OUT),
            Commands.waitSeconds(3),
            SubsystemManager.operateAuto(ShooterState.IDLE, IntakeState.ACTIVE, StorageState.OUT),
            SubsystemManager.getDrivebase().driveToPose(new APTarget(SHOOT_FIRST_ENTRANCE).withVelocity(0), 0.1, 1),
            // velocity is zero for a better steal
            SubsystemManager.getDrivebase().driveToPose(new APTarget(STEAL_END).withVelocity(0), true), 
            SubsystemManager.operateAuto(ShooterState.IDLE, IntakeState.INACTIVE, StorageState.IN),
            SubsystemManager.getDrivebase().waypointToPose(new APTarget[] {
                new APTarget(RAMP_NEXT_ENTRANCE).withVelocity(2),
                new APTarget(RAMP_NEXT_EXIT).withVelocity(2),
                new APTarget(FINAL).withVelocity(0)
            }, 0.2)
        );
    }
}