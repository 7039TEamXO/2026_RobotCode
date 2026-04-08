package frc.robot;

public enum Autos {
    Center("Center"),
    SimpleOut("Simple Out"),
    SimpleMiddle("Simple Middle"),
    SimpleOutDelay("Simple Out w/8"),
    HubSteal("Hub Steal"),
    Collect("Collect");
    
    private final String autoName;
    
    Autos(final String autoName) {
        this.autoName = autoName;
    }

    public String getAutoName() {
        return autoName;
    }
}
