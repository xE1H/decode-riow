package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Pose;

@Config
public class Points_specimen {
    public static Pose START_POSE = new Pose(10, 57, Math.toRadians(-180));

    public static Pose SCORE_PRELOAD = new Pose(37, 52, Math.toRadians(-180));

    public static Pose CONTROL_1 = new Pose(22.8, 57.7);
    public static Pose CONTROL_2 = new Pose(18.3, 7.5);

    public static Pose PICK_UP_SAMPLE_1 = new Pose(31.5, 7, Math.toRadians(0));

    public static Pose PICK_UP_SAMPLE_2 = new Pose(31.5, -3.5, Math.toRadians(0));

    public static Pose PICK_UP_SAMPLE_3 = new Pose(31.5, -7, Math.toRadians(-22));

    public static Pose PICK_UP_SPECIMEN = new Pose(17, 37, Math.toRadians(-120));

    public static Pose SCORE_SPECIMEN = new Pose(37.5, 53, Math.toRadians(-180));



    public static double rad(double deg) {
        return Math.toRadians(deg);
    }
}
