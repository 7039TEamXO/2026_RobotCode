// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import swervelib.math.Matter;

public final class Constants
{
  public static final Mode CurrentMode = RobotBase.isReal() ? Mode.REAL : (RobotBase.isSimulation() ? Mode.SIM : Mode.REPLAY);
  // public static final Mode CurrentMode = Mode.REPLAY;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  private static final TuningMode StartingTuningMode = TuningMode.INACTIVE;
  private static final LoggedNetworkBoolean IsActiveTuning = 
    new LoggedNetworkBoolean("/Tuning/[!] Is Active Tuning", StartingTuningMode == TuningMode.ACTIVE ? true : false);

  public static TuningMode GetTuningMode() {
    return StartingTuningMode == TuningMode.IGNORE ? TuningMode.IGNORE : IsActiveTuning.get() ? TuningMode.ACTIVE : TuningMode.INACTIVE;
  }

  public static enum TuningMode {
    /** Use the constants from the Tuning panel, update. */
    ACTIVE,

    /** Use the constants from the Tuning panel, don't update. */
    INACTIVE,

    /** Ignore the tuned constants. */
    IGNORE
  }

  public static final SysIdMode CurrentSysIdMode = SysIdMode.INACTIVE;

  public static enum SysIdMode {
    ACTIVE, INACTIVE
  }

  public static final LimelightSimMode CurrentLimelightSimMode = LimelightSimMode.PROPER;

  public static enum LimelightSimMode {
    PROPER, BASIC
  }

  public static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  public static final double ROBOT_MASS = 50;
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; // 100ms due to SPARK MAX velocity lag + 20ms loop time + 10ms for safety

  public static class Limelight
  {
    public static final Transform3d Main_RobotToCamera = new Transform3d(new Translation3d(-0.2675, -0.1475, 0.71), new Rotation3d(0, Math.toRadians(-7.2), Math.toRadians(-5)));
    public static final Transform3d Back_RobotToCamera = new Transform3d(new Translation3d(-0.3, 0.0, 0.2), new Rotation3d(0, Math.toRadians(-20), Math.toRadians(180))); // Temporary
    public static class CameraProperties {
      public static final int resWidth = 1280;
      public static final int resHeight = 960;
      public static final Rotation2d fovDiag = Rotation2d.fromDegrees(100);
      public static final double avgErrorPx = 0.007;
      public static final double errorStdDevPx = 0.0005;
      public static final double fps = 30;
      public static final double avgLatencyMs = 35;
      public static final double latencyStdDevMs = 5;

      public static final Matrix<N3, N1> singleTagStdDevs = VecBuilder.fill(0.3, 0.3, 4);
      public static final Matrix<N3, N1> multiTagStdDevs = VecBuilder.fill(0.5, 0.5, 2);
    }
    public static final double TAForMT1 = 0.2;
    public static final double TAForUpdate = 0.15;
  }

  public static class Operator
  {
    // Joystick Deadbands
    public static final double LEFT_X_DEADBAND  = 0.1;
    public static final double LEFT_Y_DEADBAND  = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
  }

  public static class Field {
    public static final double FIELD_WIDTH = 16.54;
    public static final double FIELD_HEIGHT = 8.07;

    public static final double HUB_X_RED = 11.915;
    public static final double HUB_Y_RED = 4.0;
    public static final double HUB_X_BLUE = 4.625;
    public static final double HUB_Y_BLUE = 4.0;

    public static final double OUTPOST_X_RED = 16.0;
    public static final double OUTPOST_Y_RED = 7.4;
    public static final double OUTPOST_X_BLUE = 0.55;
    public static final double OUTPOST_Y_BLUE = 0.65;

    public static final double TOWER_X_RED = 15.0;
    public static final double TOWER_Y_RED = 4.3;
    public static final double TOWER_X_BLUE = 1.55;
    public static final double TOWER_Y_BLUE = 3.75;

    public static final double DEPOT_X_RED = 16.0;
    public static final double DEPOT_Y_RED = 2.1;
    public static final double DEPOT_X_BLUE = 0.55;
    public static final double DEPOT_Y_BLUE = 6.0;

    public static final double RAMP_ZONE_WIDTH = 1; // really, it is half-width (distance to center)
    public static final double RAMP_ZONE_HEIGHT_START = 1.0; // and these are too
    public static final double RAMP_ZONE_HEIGHT_END = 2.5;
  }
}
