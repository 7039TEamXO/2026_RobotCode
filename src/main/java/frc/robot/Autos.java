package frc.robot;

public enum Autos {
    TestAutoRedLeft("Test Auto Red Left"),
    TestAutoRedRight("Test Auto Red Right"),
    TestAutoBlueLeft("Test Auto Blue Left"),
    TestAutoBlueRight("Test Auto Blue Right");
    
    private final String autoName;
    
    Autos(final String autoName) {
        this.autoName = autoName;
    }

    public String getAutoName() {
        return autoName;
    }
}
