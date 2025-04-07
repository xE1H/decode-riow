package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Pose;

@Config
public class Points_sample {
    public static Pose START_POSE = new Pose(10, 111.5, 0);

//    public static Pose BUCKET_HIGH_SCORE_POSE = new Pose(21, 116, rad(-45));
    public static Pose BUCKET_HIGH_SCORE_POSE = new Pose(30, 116, rad(-45));
    public static Pose BUCKET_HIGH_SCORE_TRUE_POSE = new Pose(20, 118, rad(-45));
    public static Pose FIRST_MARK_READY_POSE = new Pose(26, 118.7, 0);
    public static Pose FIRST_MARK_GRAB_POSE = new Pose(32.3, 118.7, 0);

    public static Pose SECOND_MARK_READY_POSE = new Pose(26, 129, 0);
    public static Pose SECOND_MARK_GRAB_POSE = new Pose(32.3, 129, 0);

    public static Pose THIRD_MARK_READY_POSE = new Pose(35, 120, rad(60));
    public static Pose THIRD_MARK_GRAB_POSE = new Pose(40, 125.7, rad(60));

    public static Pose SUB_PRE_BEZIER_POSE = new Pose(47,120, rad(-70));
    public static Pose SUB_PRE_PREGRAB_POSE = new Pose(60,117, rad(-90));
    public static Pose SUB_GRAB_POSE = new Pose(60,97, rad(-90));

    public static double rad(double deg) {
        return Math.toRadians(deg);
    }
}
