package org.firstinspires.ftc.teamcode.helpers.utils;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.auto.pedroPathing.tuning.FollowerConstants;

@Config
public class GlobalConfig {
    public static boolean DEBUG_MODE = true;
    public static boolean PRINT_MOTOR_CURRENT = false;

    /**
     * Avoid mutating this config variable directly, rather use the mutator setIsInvertedMotors.
     * FollowerConstants relies on knowing the latest value of this config variable
     * The variable is public for the sake of convenience and brevity.
     */
    public static boolean INVERTED_MOTORS = false;

    public static void setIsInvertedMotors(boolean isInverted) {
        INVERTED_MOTORS = isInverted;
        FollowerConstants.updateConstants();
    }

}