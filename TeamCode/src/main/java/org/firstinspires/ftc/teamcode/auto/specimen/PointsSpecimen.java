package org.firstinspires.ftc.teamcode.auto.specimen;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Pose;

@Config
public class PointsSpecimen {

    public static Pose START_POSE = new Pose(10, 57, Math.toRadians(0));

    public static Pose SCORE_PRELOAD_AND_SUB_PICKUP = new Pose(35.85, 55, Math.toRadians(0));

    public static Pose PICK_UP_SAMPLE_1 = new Pose(29, 14.5, Math.toRadians(0));

    public static Pose PICK_UP_SAMPLE_2 = new Pose(29, 5, Math.toRadians(0));

    public static Pose PICK_UP_SAMPLE_3 = new Pose(29, 4, Math.toRadians(-21.75));

    public static Pose DEPOSIT_SAMPLE_3_START = new Pose(29,16, Math.toRadians(0));

    public static Pose DEPOSIT_SAMPLE_3_END = new Pose(25,16, Math.toRadians(0));


    public static Pose SCORE_SECOND_SPECIMEN = new Pose(34.25, 57, Math.toRadians(0));

    public static Pose DRIVE_BACK = new Pose(28, 57, Math.toRadians(0));

    public static Pose PICK_UP_SPECIMENS_FROM_HUMAN_PLAYER = new Pose(22, 40, Math.toRadians(-120));

    public static Pose SCORE_SPECIMEN_BACK = new Pose(31, 61, Math.toRadians(-180));

    public static Pose SCORE_SPECIMEN_BACK_4 = new Pose(31, 65, Math.toRadians(-180));

    public static Pose SCORE_SPECIMEN_BACK_5 = new Pose(31, 70, Math.toRadians(-180));

    public static Pose SUB_GRAB_SPEC = new Pose(64, 120 - 95.5, Math.toRadians(90));
    public static Pose SUB_GRAB_SPEC_CONTROL = new Pose(55.2, 12, Math.toRadians(0));

    public static Pose SUB_GRAB_SPEC_DEPOSIT = new Pose(25, 14.5, 0);
}
