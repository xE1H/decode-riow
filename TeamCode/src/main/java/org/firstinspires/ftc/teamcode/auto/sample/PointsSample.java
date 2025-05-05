package org.firstinspires.ftc.teamcode.auto.sample;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Pose;

@Config
public class PointsSample {
    public static Pose START_POSE = new Pose(10, 111.5, 0);

    public static Pose BUCKET_HIGH_SCORE_POSE = new Pose(24, 118.5, rad(-45));

    public static Pose BUCKET_HIGH_SCORE_POSE_SUB = new Pose(24.5, 118, rad(-45));

    public static Pose FIRST_MARK_GRAB = new Pose(29, 118.4, 0);

    public static Pose SECOND_MARK_GRAB = new Pose(FIRST_MARK_GRAB.getX(), FIRST_MARK_GRAB.getY() + 9.75, 0);

    public static Pose THIRD_MARK_GRAB = new Pose(32.5, 130, rad(30));

    public static Pose SUB_GRAB_0 = new Pose(55.2, 111.5435, Math.toRadians(-45));

    public static Pose SUB_GRAB_CONTROL_1 = new Pose(59.3124, 107.4);

    public static Pose SUB_GRAB_CONTROL_2 = new Pose(63.775, 104.6);

    public static Pose SUB_GRAB = new Pose(64, 95.5, Math.toRadians(-90));

    public static Pose[] GRAB_POSES = {FIRST_MARK_GRAB, SECOND_MARK_GRAB, THIRD_MARK_GRAB};

    public static double rad(double deg) {
        return Math.toRadians(deg);
    }
}
