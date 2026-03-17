package frc.robot;

public enum Autos {
    CollectRedLeft("Collect (Red Left)"),
    CollectRedRight("Collect (Red Right)"),
    CollectBlueLeft("Collect (Blue Left)"),
    CollectBlueRight("Collect (Blue Right)"),
    ShootRedLeft("Shoot (Red Left)"),
    ShootRedRight("Shoot (Red Right)"),
    ShootBlueLeft("Shoot (Blue Left)"),
    ShootBlueRight("Shoot (Blue Right)");
    
    private final String autoName;
    
    Autos(final String autoName) {
        this.autoName = autoName;
    }

    public String getAutoName() {
        return autoName;
    }
}
