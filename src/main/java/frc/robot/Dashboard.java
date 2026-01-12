package frc.robot;

import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.SubsystemManager;

public class Dashboard {
    private final static SendableChooser<String> autoChooser = new SendableChooser<>();

    @SuppressWarnings("unused")
    private static HttpCamera limelightCamera;

    public static void init() {
        Autos[] states = Autos.values();
        for (int i = 0; i < states.length; i++) {
            Autos state = states[i];
            String name_state = state.getAutoName();
            if (i == 0) {
                autoChooser.setDefaultOption(name_state, name_state);
            }
            autoChooser.addOption(name_state, name_state);
        }

        limelightCamera = new HttpCamera("limelight", "http://10.70.39.202:5801/");

        // -----------------------
          
        SmartDashboard.putData("Autos", autoChooser);

        update();
    }

    public static void update() {
        SmartDashboard.postListenerTask(() -> SmartDashboard.putNumber("X", SubsystemManager.getDrivebase().getPose().getX()));
        SmartDashboard.postListenerTask(() -> SmartDashboard.putNumber("Y", SubsystemManager.getDrivebase().getPose().getY()));
        SmartDashboard.postListenerTask(() -> SmartDashboard.putNumber("Rotation", SubsystemManager.getDrivebase().getPose().getRotation().getDegrees()));
        SmartDashboard.postListenerTask(() -> SmartDashboard.putNumber("Front Left", SubsystemManager.getDrivebase().getSwerveDriveConfiguration().modules[0].getPosition().angle.getDegrees()));
        SmartDashboard.postListenerTask(() -> SmartDashboard.putNumber("Front Right", SubsystemManager.getDrivebase().getSwerveDriveConfiguration().modules[1].getPosition().angle.getDegrees()));
        SmartDashboard.postListenerTask(() -> SmartDashboard.putNumber("Back Left", SubsystemManager.getDrivebase().getSwerveDriveConfiguration().modules[2].getPosition().angle.getDegrees()));
        SmartDashboard.postListenerTask(() -> SmartDashboard.putNumber("Back Right", SubsystemManager.getDrivebase().getSwerveDriveConfiguration().modules[3].getPosition().angle.getDegrees()));

        SmartDashboard.updateValues();
    }
        
    public static String getSelectedAutonomy() {
        return autoChooser.getSelected();
    }
}