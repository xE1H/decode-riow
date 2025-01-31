package org.firstinspires.ftc.teamcode.subsystems.arm;

public class ArmLowState {
    public static boolean isLowScoring = false;
    private static boolean wasJustToggled = false;

    public static void toggle() {
        isLowScoring = !isLowScoring;
        wasJustToggled = true;
    }

    public static boolean get() {
        return isLowScoring;
    }

    public static boolean wasJustToggled() {
        boolean oldToggled = wasJustToggled;
        wasJustToggled = false;
        return oldToggled;
    }
}
