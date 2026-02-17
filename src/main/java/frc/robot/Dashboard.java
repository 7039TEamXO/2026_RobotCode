package frc.robot;

import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.SubsystemManager;

public class Dashboard {
    private final static SendableChooser<String> autoChooser = new SendableChooser<>();

    private static NetworkTableEntry acceptAngleChanges;
    private static NetworkTableEntry setAngleValue;

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

        SmartDashboard.putBoolean("Accept Angle Changes", false);
        acceptAngleChanges = SmartDashboard.getEntry("Accept Angle Changes");

        SmartDashboard.putNumber("Set Angle Value", 0);
        setAngleValue = SmartDashboard.getEntry("Set Angle Value");

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

        SmartDashboard.postListenerTask(() -> SmartDashboard.putString("Shooter State", SubsystemManager.getShooterState().name()));
        SmartDashboard.postListenerTask(() -> SmartDashboard.putString("Conveyor State", SubsystemManager.getConveyorState().name()));
        SmartDashboard.postListenerTask(() -> SmartDashboard.putString("Intake State", SubsystemManager.getIntakeState().name()));
        SmartDashboard.postListenerTask(() -> SmartDashboard.putString("Storage State", SubsystemManager.getStorageState().name()));

        SmartDashboard.postListenerTask(() -> SmartDashboard.putNumber("Shift Timer", getMatchTimer()));
        SmartDashboard.postListenerTask(() -> SmartDashboard.putString("Match State", whetherCanShoot() ? "Active" : "Inactive"));
        SmartDashboard.postListenerTask(() -> SmartDashboard.putString("Shift Color", getGameState().toHexString()));
        SmartDashboard.postListenerTask(() -> SmartDashboard.putString("Autonomy Winner", getAutonomyWinner()));
        SmartDashboard.postListenerTask(() -> SmartDashboard.putString("Shift Name", getGameStateName()));
        SmartDashboard.postListenerTask(() -> SmartDashboard.putString("Flash Bar", getFlashbarColor().toHexString()));

        SmartDashboard.updateValues();
    }

    public static double getMatchTimer() {
        double matchTimer = Timer.getMatchTime();
        if(!Robot.isAuto()) {
            if(matchTimer >= 130) matchTimer -= 130;
            else if(matchTimer >= 105) matchTimer -= 105;
            else if(matchTimer >= 80) matchTimer -= 80;
            else if(matchTimer >= 55) matchTimer -= 55;
            else if(matchTimer >= 30) matchTimer -= 30;
        }

        return matchTimer;
    }

    public static Boolean whetherCanShoot() {
        String message = DriverStation.getGameSpecificMessage();
        char state = 'E'; // Empty
        if(message.length() > 0) {
            state = DriverStation.getGameSpecificMessage().charAt(0);
        }
        double matchTimer = Timer.getMatchTime();

        int turn = -1;

        if(!Robot.isAuto()) {
            if(matchTimer >= 130) turn = -1;
            else if(matchTimer >= 105) turn = 0;
            else if(matchTimer >= 80) turn = 1;
            else if(matchTimer >= 55) turn = 0;
            else if(matchTimer >= 30) turn = 1;
        }

        Boolean answer = true;
        int alliance = RobotContainer.teamColorIsBlue();

        if(turn != -1) {
            if(state == 'R') {
                if(alliance == 1) { // Blue
                    answer = turn == 0;
                } else { // Red
                    answer = turn == 1;
                }
            } else if(state == 'B') {
                if(alliance == 1) { // Blue
                    answer = turn == 1;
                } else { // Red
                    answer = turn == 0;
                }
            }
        }

        return answer;
    }

    public static Color getGameState() {
        String message = DriverStation.getGameSpecificMessage();
        char state = 'E'; // Empty
        if(message.length() > 0) {
            state = DriverStation.getGameSpecificMessage().charAt(0);
        }
        double matchTimer = Timer.getMatchTime();

        int turn = -1;
        Color color = Color.kLimeGreen; // Auto as the default

        if(!Robot.isAuto()) {
            if(matchTimer >= 130) { 
                turn = -1;
                color = Color.kMediumPurple;
            }
            else if(matchTimer >= 105) turn = 0;
            else if(matchTimer >= 80) turn = 1;
            else if(matchTimer >= 55) turn = 0;
            else if(matchTimer >= 30) turn = 1;
            else {
                color = Color.kYellow;
            }
        }

        if(turn != -1) {
            if(state == 'R') {
                if(turn == 0) { // Blue
                    color = Color.kBlue;
                } else { // Red
                    color = Color.kRed;
                }
            } else if(state == 'B') {
                if(turn == 0) { // Blue
                    color = Color.kRed;
                } else { // Red
                    color = Color.kBlue;
                }
            }
        }

        return color;
    }

    public static String getGameStateName() {
        String message = DriverStation.getGameSpecificMessage();
        char state = 'E'; // Empty
        if(message.length() > 0) {
            state = DriverStation.getGameSpecificMessage().charAt(0);
        }
        double matchTimer = Timer.getMatchTime();

        int turn = -1;
        String turnTime = "1";
        String name = "Autonomy"; // Auto as the default

        if(!Robot.isAuto()) {
            if(matchTimer >= 130) { 
                turn = -1;
                name = "Transition Shift";
            }
            else if(matchTimer >= 105) turn = 0;
            else if(matchTimer >= 80) turn = 1;
            else if(matchTimer >= 55) {
                turn = 0;
                turnTime = "2";
            }
            else if(matchTimer >= 30) {
                turn = 1;
                turnTime = "2";
            }
            else {
                name = "Endgame";
            }
        }

        if(turn != -1) {
            if(state == 'R') {
                if(turn == 0) { // Blue
                    name = "Blue Shift ".concat(turnTime);
                } else { // Red
                    name = "Red Shift ".concat(turnTime);
                }
            } else if(state == 'B') {
                if(turn == 0) { // Blue
                    name = "Red Shift ".concat(turnTime);
                } else { // Red
                    name = "Blue Shift ".concat(turnTime);
                }
            }
        }

        return name;
    }

    public static String getAutonomyWinner() {
        String message = DriverStation.getGameSpecificMessage();
        char state = 'E'; // Empty
        if(message.length() > 0) {
            state = DriverStation.getGameSpecificMessage().charAt(0);
        }

        Color color = Color.kYellow;
        if(state == 'B') color = Color.kBlue;
        else if(state == 'R') color = Color.kRed;

        if(Timer.getMatchTime() < 130 || Robot.isAuto()) {
            return "";
        }
        return color.toHexString();
    }

    public static Color getFlashbarColor() {
        Color allianceColor = RobotContainer.teamColorIsBlue() == 1 ? Color.kBlue : Color.kRed;

        Color color = allianceColor;
        double matchTimer = Timer.getMatchTime();

        if(!Robot.isAuto()) {
            if((matchTimer >= 130 && matchTimer <= 135) ||
                (matchTimer >= 105 && matchTimer <= 110) ||
                (matchTimer >= 80 && matchTimer <= 85) ||
                (matchTimer >= 55 && matchTimer <= 60) ||
                (matchTimer >= 30 && matchTimer <= 35) ||
                (matchTimer <= 5)) {
                    if(1000 * matchTimer % 200 <= 100) {
                        color = Color.kWhite;
                    } else {
                        color = allianceColor;
                    }
            }
        }

        return color;
    }
        
    public static String getSelectedAutonomy() {
        return autoChooser.getSelected();
    }

    public static Boolean getAcceptAngleChanges() {
        return acceptAngleChanges.getBoolean(false);
    }

    public static double getAngleValue() {
        return setAngleValue.getDouble(0);
    }
}