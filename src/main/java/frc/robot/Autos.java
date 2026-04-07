package frc.robot;

public enum Autos {
    Center("Center"),
    ShortCollect("Short Collect"),
    ShortShoot("Short Shoot"),
    DelayShortShoot("Short Shoot w/ Delay"),
    ChessShoot("Chess Shoot"),
    StealShoot("Steal Shoot");
    
    private final String autoName;
    
    Autos(final String autoName) {
        this.autoName = autoName;
    }

    public String getAutoName() {
        return autoName;
    }
}
