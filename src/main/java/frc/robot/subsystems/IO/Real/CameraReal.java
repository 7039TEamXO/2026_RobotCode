package frc.robot.subsystems.IO.Real;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;
import frc.robot.Constants.Limelight;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.IO.CameraIO;
import frc.robot.utils.LimelightHelpers;

public class CameraReal implements CameraIO {
    private final NetworkTable limelightTable;
    private final NetworkTable limelightTableBack;
    private final int allValidIDs[];

    private double Main_AngleFromMT1;
    private boolean Main_DoRejectUpdate;
    private Pose2d Main_EstimatedPose;
    private double Main_TimestampSeconds;
    private Matrix<N3, N1> Main_StdDevs;

    private double Back_AngleFromMT1;
    private boolean Back_DoRejectUpdate;
    private Pose2d Back_EstimatedPose;
    private double Back_TimestampSeconds;
    private Matrix<N3, N1> Back_StdDevs;

    public CameraReal() {
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
        limelightTableBack = NetworkTableInstance.getDefault().getTable("limelight_back");
        allValidIDs = new int[]{1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32};

        Main_AngleFromMT1 = 0;

        Main_DoRejectUpdate = true;
        Main_EstimatedPose = new Pose2d();
        Main_TimestampSeconds = 0;
        Main_StdDevs = VecBuilder.fill(0.1, 0.1, 0.01);
    }

    public void init() {
        setLimelightConfiguration();
        setBackLimelightConfiguration();
    }

    private void setLimelightConfiguration() {
        LimelightHelpers.SetFiducialIDFiltersOverride("limelight", allValidIDs);
        // Process at half resolution for improved framerate and reduced range
        LimelightHelpers.SetFiducialDownscalingOverride("limelight", 2.0f); 
        LimelightHelpers.setCameraPose_RobotSpace("limelight", 
            Constants.Limelight.Main_RobotToCamera.getX(),                   // Forward offset (meters)
            Constants.Limelight.Main_RobotToCamera.getY(),                   // Side offset (meters)
            Constants.Limelight.Main_RobotToCamera.getZ(),                   // Height offset (meters)
            Constants.Limelight.Main_RobotToCamera.getRotation().getX(),     // Roll (degrees)
            Constants.Limelight.Main_RobotToCamera.getRotation().getY(),     // Pitch (degrees)
            Constants.Limelight.Main_RobotToCamera.getRotation().getZ()      // Yaw (degrees)
        );
    }

    private void setBackLimelightConfiguration() {
        LimelightHelpers.SetFiducialIDFiltersOverride("limelight_back", allValidIDs);
        // Process at half resolution for improved framerate and reduced range
        LimelightHelpers.SetFiducialDownscalingOverride("limelight_back", 2.0f); 
        LimelightHelpers.setCameraPose_RobotSpace("limelight_back", 
            Constants.Limelight.Back_RobotToCamera.getX(),                   // Forward offset (meters)
            Constants.Limelight.Back_RobotToCamera.getY(),                   // Side offset (meters)
            Constants.Limelight.Back_RobotToCamera.getZ(),                   // Height offset (meters)
            Constants.Limelight.Back_RobotToCamera.getRotation().getX(),     // Roll (degrees)
            Constants.Limelight.Back_RobotToCamera.getRotation().getY(),     // Pitch (degrees)
            Constants.Limelight.Back_RobotToCamera.getRotation().getZ()      // Yaw (degrees)
        );
    }

    public void updateInputs(CameraIOInputs inputs) {
        inputs.Main_DoRejectUpdate = Main_DoRejectUpdate;
        inputs.Main_EstimatedPose = Main_EstimatedPose;
        inputs.Main_TimestampSeconds = Main_TimestampSeconds;
        inputs.Main_StdDevs = Main_StdDevs.getData();

        inputs.Main_HasTarget = Main_HasTarget();
        inputs.Main_MainAprilTagID = Main_GetMainAprilTagID();
        inputs.Main_TA = LimelightHelpers.getTA("limelight");
        inputs.Main_MT1 = Main_AngleFromMT1;

        inputs.Back_DoRejectUpdate = Back_DoRejectUpdate;
        inputs.Back_EstimatedPose = Back_EstimatedPose;
        inputs.Back_TimestampSeconds = Back_TimestampSeconds;
        inputs.Back_StdDevs = Back_StdDevs.getData();

        inputs.Back_HasTarget = Back_HasTarget();
        inputs.Back_MainAprilTagID = Back_GetMainAprilTagID();
        inputs.Back_TA = LimelightHelpers.getTA("limelight_back");
        inputs.Back_MT1 = Back_AngleFromMT1;
    }

    public void periodic() {
        Main_Periodic();
        Back_Periodic();
    }

    private void Main_Periodic() {
        Main_DoRejectUpdate = false;

        double suppliedAngle = SubsystemManager.getDrivebase().getHeading().getDegrees();

        // Temporary replacement due to Pigeon's unpredictability
        if(LimelightHelpers.getTA("limelight") > Limelight.TAForMT1) { 
            Double[] inputArray = limelightTable.getEntry("botpose_wpiblue").getDoubleArray(new Double[]{});
            if(inputArray.length > 5) {
                Main_AngleFromMT1 = limelightTable.getEntry("botpose_wpiblue").getDoubleArray(new Double[]{})[5];
                suppliedAngle = Main_AngleFromMT1;
            }
        }
        
        LimelightHelpers.SetRobotOrientation("limelight", suppliedAngle, 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        double[] currentStdDevs = limelightTable.getEntry("stddevs").getDoubleArray(new double[12]);
      
        if (mt2 != null) { // if our angular velocity is greater than 360 degrees per second, ignore vision updates
            if(Math.abs(SubsystemManager.getDrivebase().getRobotVelocity().omegaRadiansPerSecond) > 2 * Math.PI)
                Main_DoRejectUpdate = true;
        
            else if(mt2.tagCount == 0)
                Main_DoRejectUpdate = true;
        
            else if (Main_HasTarget() == false)
                Main_DoRejectUpdate = true;
        
            if(!Main_DoRejectUpdate) {
                Main_EstimatedPose = mt2.pose;
                Main_TimestampSeconds = mt2.timestampSeconds;
                Main_StdDevs = VecBuilder.fill(currentStdDevs[6], currentStdDevs[7], currentStdDevs[5]);
            }
        } 
    }

    private void Back_Periodic() {
        Back_DoRejectUpdate = false;

        double suppliedAngle = SubsystemManager.getDrivebase().getHeading().getDegrees();

        // Temporary replacement due to Pigeon's unpredictability
        if(LimelightHelpers.getTA("limelight_back") > Limelight.TAForMT1) { 
            Double[] inputArray = limelightTableBack.getEntry("botpose_wpiblue").getDoubleArray(new Double[]{});
            if(inputArray.length > 5) {
                Back_AngleFromMT1 = limelightTableBack.getEntry("botpose_wpiblue").getDoubleArray(new Double[]{})[5];
                suppliedAngle = Back_AngleFromMT1;
            }
        }
        
        LimelightHelpers.SetRobotOrientation("limelight_back", suppliedAngle, 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight_back");
        double[] currentStdDevs = limelightTableBack.getEntry("stddevs").getDoubleArray(new double[12]);
      
        if (mt2 != null) { // if our angular velocity is greater than 360 degrees per second, ignore vision updates
            if(Math.abs(SubsystemManager.getDrivebase().getRobotVelocity().omegaRadiansPerSecond) > 2 * Math.PI)
                Back_DoRejectUpdate = true;
        
            else if(mt2.tagCount == 0)
                Back_DoRejectUpdate = true;
        
            else if (Main_HasTarget() == false)
                Back_DoRejectUpdate = true;
        
            if(!Back_DoRejectUpdate) {
                Back_EstimatedPose = mt2.pose;
                Back_TimestampSeconds = mt2.timestampSeconds;
                Back_StdDevs = VecBuilder.fill(currentStdDevs[6], currentStdDevs[7], currentStdDevs[5]);
            }
        } 
    }

    private boolean Main_HasTarget() {
        return LimelightHelpers.getTV("limelight");
    }

    private int Main_GetMainAprilTagID() {
        return (int)LimelightHelpers.getFiducialID("limelight");
    }

    private boolean Back_HasTarget() {
        return LimelightHelpers.getTV("limelight_back");
    }

    private int Back_GetMainAprilTagID() {
        return (int)LimelightHelpers.getFiducialID("limelight_back");
    }

    @Override
    public void close() throws Exception {
        // ...
    }
}
