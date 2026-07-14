package org.firstinspires.ftc.teamcode;

public enum FarNearLocation {
    FAR,
    NEAR,
    INSTANCE;

    /**
     * Standard way to toggle the current spike location.
     * @param buttonPressed button result passed by the caller
     * @param currentLocation what the caller considers the current location
     * @return
     */
    public FarNearLocation toggleLocation(boolean buttonPressed, FarNearLocation currentLocation) {
        if (buttonPressed) {
            if (currentLocation == FarNearLocation.FAR) {
                currentLocation = FarNearLocation.NEAR;
            }
            else {
                currentLocation = FarNearLocation.FAR;
            }
        }

        return currentLocation;
    }
}
