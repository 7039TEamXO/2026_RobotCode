package frc.robot;

public enum Autos {
    CollectRedLeft("Collect (Red Left)"),
    CollectRedRight("Collect (Red Right)"),
    CollectBlueLeft("Collect (Blue Left)"),
    CollectBlueRight("Collect (Blue Right)"),
    CenterRedLeft("Center (Red)"),
    CenterRedRight("Center (Blue)"),
    ShortShootRedLeft("Short Shoot (Red Left)"),
    ShortShootRedRight("Short Shoot (Red Right)"),
    ShortShootBlueLeft("Short Shoot (Blue Left)"),
    ShortShootBlueRight("Short Shoot (Blue Right)"),
    LongShootRedLeft("Long Shoot (Red Left)"),
    LongShootRedRight("Long Shoot (Red Right)"),
    LongShootBlueLeft("Long Shoot (Blue Left)"),
    LongShootBlueRight("Long Shoot (Blue Right)");
    
    private final String autoName;
    
    Autos(final String autoName) {
        this.autoName = autoName;
    }

    public String getAutoName() {
        return autoName;
    }
}
