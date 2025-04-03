package org.firstinspires.ftc.teamcode.subsystems.limelight;

import com.acmerobotics.dashboard.config.Config;

@Config
public class LimelightConfiguration {
    public static String NAME = "limelight";

    public static int BLUE_PIPELINE = 1;
    public static int RED_PIPELINE = 2;
    public static int NOTHING_PIPELINE = 0; // Have this pipeline to not have to run the CPU of
    // the limelight to 100% all the time when not actually using it
    public static int ANGLE_EST_PIPELINE = 4;

    public static int UPDATE_TIMEOUT = 100; // ms;
    // the amount of time after which the data will be marked as stale

    public static int POLL_RATE_HZ = 28; // FPS is around 14, so make it poll 2x as fast

    public static double POS_X = -117 / 25.4; // inches right from robot center, -117mm
    public static double POS_Y = -183.6 / 25.4; // inches from the front of the robot 228.27mm
    public static double POS_Z = 234 / 25.4; // inches from the ground, 166.5mm
    public static double TILT_ANGLE = 90 + 52; // degrees to the ground (positive is to the ground)

    // This was calibrated with 3df
    public static double FX = 599.718;
    public static double FY = 599.718;
    public static double CX = 304.177;
    public static double CY = 242.588;


}
