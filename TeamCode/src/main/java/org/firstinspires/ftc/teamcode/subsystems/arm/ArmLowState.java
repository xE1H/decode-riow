package org.firstinspires.ftc.teamcode.subsystems.arm;

public class ArmLowState {
    public static boolean isLowScoring = false;

    public static void toggle() {
        isLowScoring = !isLowScoring;
    }

    public static boolean get() {
        return isLowScoring;
    }
}
