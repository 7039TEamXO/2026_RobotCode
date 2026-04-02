package frc.robot;

public enum Autos {
    Center("Center"),
    ShortCollect("Short Collect"),
    DelayShortCollect("Short Collect w/ Delay"),
    LongCollect("Long Collect"),
    ShortShoot("Short Shoot"),
    DelayShortShoot("Short Shoot w/ Delay"),
    LongShoot("Long Shoot");
    
    private final String autoName;
    
    Autos(final String autoName) {
        this.autoName = autoName;
    }

    public String getAutoName() {
        return autoName;
    }
}
