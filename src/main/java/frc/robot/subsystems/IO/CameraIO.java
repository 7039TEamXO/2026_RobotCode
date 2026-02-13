package frc.robot.subsystems.IO;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import edu.wpi.first.math.geometry.Pose2d;

public interface CameraIO extends AutoCloseable {
    public static class CameraIOInputs implements LoggableInputs {
        public boolean Main_DoRejectUpdate = true;
        public Pose2d Main_EstimatedPose = new Pose2d();
        public double Main_TimestampSeconds = 0.0;
        public double[] Main_StdDevs = new double[]{ 0.1, 0.1, 0.01 };

        public boolean Main_HasTarget = false;
        public int Main_MainAprilTagID = -1;
        public double Main_TA = 0;
        public double Main_MT1 = 0;

        public boolean Back_DoRejectUpdate = true;
        public Pose2d Back_EstimatedPose = new Pose2d();
        public double Back_TimestampSeconds = 0.0;
        public double[] Back_StdDevs = new double[]{ 0.1, 0.1, 0.01 };

        public boolean Back_HasTarget = false;
        public int Back_MainAprilTagID = -1;
        public double Back_TA = 0;
        public double Back_MT1 = 0;

        public void toLog(LogTable table) {
            table.put("Main DoRejectUpdate", Main_DoRejectUpdate);
            table.put("Main Estimated Pose", Main_EstimatedPose);
            table.put("Main TimestampSeconds", Main_TimestampSeconds);
            table.put("Main StdDevs", Main_StdDevs);

            table.put("Main HasTarget", Main_HasTarget);
            table.put("Main MainAprilTagID", Main_MainAprilTagID);
            table.put("Main TA", Main_TA);
            table.put("Main MT1", Main_MT1);

            table.put("Back DoRejectUpdate", Back_DoRejectUpdate);
            table.put("Back Estimated Pose", Back_EstimatedPose);
            table.put("Back TimestampSeconds", Back_TimestampSeconds);
            table.put("Back StdDevs", Back_StdDevs);

            table.put("Back HasTarget", Back_HasTarget);
            table.put("Back MainAprilTagID", Back_MainAprilTagID);
            table.put("Back TA", Back_TA);
            table.put("Back MT1", Back_MT1);
        }

        public void fromLog(LogTable table) {
            Main_DoRejectUpdate = table.get("Main DoRejectUpdate", true);
            Main_EstimatedPose = table.get("Main EstimatedPose", new Pose2d());
            Main_TimestampSeconds = table.get("Main TimestampSeconds", 0.0);
            Main_StdDevs = table.get("Main StdDevs", new double[]{ 0.1, 0.1, 0.01 });

            Main_HasTarget = table.get("Main HasTarget", false);
            Main_MainAprilTagID = table.get("Main MainAprilTagID", -1);
            Main_TA = table.get("Main TA", 0);
            Main_MT1 = table.get("Main MT1", 0);

            Back_DoRejectUpdate = table.get("Back DoRejectUpdate", true);
            Back_EstimatedPose = table.get("Back EstimatedPose", new Pose2d());
            Back_TimestampSeconds = table.get("Back TimestampSeconds", 0.0);
            Back_StdDevs = table.get("Back StdDevs", new double[]{ 0.1, 0.1, 0.01 });

            Back_HasTarget = table.get("Back HasTarget", false);
            Back_MainAprilTagID = table.get("Back MainAprilTagID", -1);
            Back_TA = table.get("Back TA", 0);
            Back_MT1 = table.get("Back MT1", 0);
        }
    }

    public void init();

    public void updateInputs(CameraIOInputs inputs);

    public void periodic();
}
