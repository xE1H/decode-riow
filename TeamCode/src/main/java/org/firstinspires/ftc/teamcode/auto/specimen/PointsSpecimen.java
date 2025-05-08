package org.firstinspires.ftc.teamcode.auto.specimen;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Pose;

@Config
public class PointsSpecimen {

    public static Pose START_POSE = new Pose(10, 57, Math.toRadians(0));

    public static Pose SCORE_PRELOAD_AND_SUB_PICKUP = new Pose(37.5, 54.5, Math.toRadians(0));

    public static Pose PICK_UP_SAMPLE_1 = new Pose(29, 15.6, Math.toRadians(0));

    public static Pose PICK_UP_SAMPLE_2 = new Pose(29, 5.1, Math.toRadians(0));

    public static Pose PICK_UP_SAMPLE_3 = new Pose(29, 3.2, Math.toRadians(-22));

    public static Pose DEPOSIT_SAMPLE_3_START = new Pose(29, 16, Math.toRadians(0));

    public static Pose DEPOSIT_SAMPLE_3_END = new Pose(24.5, 16, Math.toRadians(0));

    public static Pose SCORE_SECOND_SPECIMEN = new Pose(34.25, 57.5, Math.toRadians(0));

    public static Pose DRIVE_BACK = new Pose(28, 57.5, Math.toRadians(0));

    public static Pose PICK_UP_SPECIMENS_FROM_HUMAN_PLAYER = new Pose(22, 40, Math.toRadians(-120));

    public static Pose SCORE_SPECIMEN_BACK = new Pose(31.3, 61, Math.toRadians(-180));

    public static double DELTA = 2.75; //Y OFFSET FOR 4TH AND 5TH SPECIMEN

    public static Pose SUB_GRAB_SPEC = new Pose(64, 120 - 95.5, Math.toRadians(90));
    public static Pose SUB_GRAB_SPEC_CONTROL = new Pose(55.2, 17, Math.toRadians(0));

    public static Pose SUB_GRAB_SPEC_DEPOSIT = new Pose(23, 19.5, 0);
}
