package frc.robot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.VecBuilder;
import frc.robot.subsystems.IO.CameraIO;
import frc.robot.subsystems.IO.CameraIO.CameraIOInputs;

public class Limelight {
    private static CameraIO io;
    private static CameraIOInputs inputs = new CameraIOInputs();

    public static void init(CameraIO _io) {
        io = _io;

        io.init();
        io.updateInputs(inputs);
        Logger.processInputs("Limelight", inputs);
    }

    public static Object[] update() {
        io.periodic();
        io.updateInputs(inputs);
        Logger.processInputs("Limelight", inputs);

        Object[] submission = null;

        if(!inputs.Main_DoRejectUpdate) {
            submission = new Object[]{ inputs.Main_EstimatedPose, inputs.Main_TimestampSeconds, VecBuilder.fill(inputs.Main_StdDevs[0], inputs.Main_StdDevs[1], inputs.Main_StdDevs[2]), inputs.Main_TA };
        }

        if(!inputs.Back_DoRejectUpdate && (inputs.Main_DoRejectUpdate || inputs.Back_TA > inputs.Main_TA)) {
            submission = new Object[]{ inputs.Back_EstimatedPose, inputs.Back_TimestampSeconds, VecBuilder.fill(inputs.Back_StdDevs[0], inputs.Back_StdDevs[1], inputs.Back_StdDevs[2]), inputs.Back_TA };
        }
        
        return submission;  
    }

    public static boolean hasTarget() {
        return inputs.Main_HasTarget || inputs.Back_HasTarget;
    }

    public static double getTA() {
        return Math.max(inputs.Main_TA, inputs.Back_TA);
    }

    public static boolean filterTargetByTA() {
        return getTA() > 0.15 && hasTarget();
    }
}