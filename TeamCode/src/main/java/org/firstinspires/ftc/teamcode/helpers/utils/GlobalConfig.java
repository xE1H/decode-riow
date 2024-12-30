package org.firstinspires.ftc.teamcode.helpers.utils;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.auto.pedroPathing.tuning.FollowerConstants;

@Config
public class GlobalConfig {
    public static boolean DEBUG_MODE = false;
    public static boolean PRINT_MOTOR_CURRENT = false;

    /**
     * Avoid mutating this config variable directly, rather use the mutator setIsInvertedMotors.
     * FollowerConstants relies on knowing the latest value of this config variable
     * The variable is public for the sake of convenience and brevity.
     */
    public static boolean INVERTED_MOTORS = false;

    public static void setIsInvertedMotors(boolean isInverted) {
        INVERTED_MOTORS = isInverted;
        setIsInvertedEncoders(isInverted);
        setIsInvertedOffsets(isInverted);
        FollowerConstants.updateConstants();
    }

    public static boolean INVERTED_ENCODERS = false;

    public static void setIsInvertedEncoders(boolean isInverted){
        INVERTED_ENCODERS = isInverted;
    }

    public static boolean INVERTED_OFFSETS = false;

    public static void setIsInvertedOffsets(boolean isInverted){
        INVERTED_OFFSETS = isInverted;
    }

}