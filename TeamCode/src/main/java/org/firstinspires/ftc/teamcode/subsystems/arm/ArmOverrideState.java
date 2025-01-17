package org.firstinspires.ftc.teamcode.subsystems.arm;

public class ArmOverrideState {
    private static boolean override = false;

    public static void set(boolean overrideValue) {
        System.out.println("Setting arm override to " + overrideValue);
        override = overrideValue;
    }

    public static boolean get() {
        return override;
    }
}
