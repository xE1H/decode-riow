package org.firstinspires.ftc.teamcode.auto.specimen;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Pose;

@Config
public class PointsSpecimen {
    private static double OFFSET = 8.5;

    public static Pose START_POSE = new Pose(10, 57 + OFFSET, Math.toRadians(0)); //63.5

    public static Pose SCORE_PRELOAD_AND_SUB_PICKUP = new Pose(35.85, 54.5 + OFFSET, Math.toRadians(0));

    public static Pose PICK_UP_SAMPLE_1 = new Pose(29, 15.5 + OFFSET, Math.toRadians(0));

    public static Pose PICK_UP_SAMPLE_2 = new Pose(29, 5.1 + OFFSET, Math.toRadians(0));

    public static Pose PICK_UP_SAMPLE_3 = new Pose(29, 3.9 + OFFSET, Math.toRadians(-22));

    public static Pose DEPOSIT_SAMPLE_3_START = new Pose(29, 16 + OFFSET, Math.toRadians(0));

    public static Pose DEPOSIT_SAMPLE_3_END = new Pose(25, 16 + OFFSET, Math.toRadians(0));

    public static Pose SCORE_SECOND_SPECIMEN = new Pose(34.25, 57.5 + OFFSET, Math.toRadians(0));

    public static Pose DRIVE_BACK = new Pose(28, 57.5 + OFFSET, Math.toRadians(0));

    public static Pose PICK_UP_SPECIMENS_FROM_HUMAN_PLAYER = new Pose(22, 40 + OFFSET, Math.toRadians(-120));

    public static Pose SCORE_SPECIMEN_BACK = new Pose(31, 61 + OFFSET, Math.toRadians(-180));

    public static Pose SCORE_SPECIMEN_BACK_4 = new Pose(31, 65 + OFFSET, Math.toRadians(-180));

    public static Pose SCORE_SPECIMEN_BACK_5 = new Pose(31, 70 + OFFSET, Math.toRadians(-180));

    public static Pose SUB_GRAB_SPEC = new Pose(64, 120 - 95.5, Math.toRadians(90));
    public static Pose SUB_GRAB_SPEC_CONTROL = new Pose(27, 20, Math.toRadians(0));

    public static Pose SUB_GRAB_SPEC_DEPOSIT = new Pose(25, 19.5, 0);

    public static Pose SUB_GRAB_SPEC_DEPOSIT_TRANSITION_CONTROL = new Pose(30, 25.5, 0);
    public static Pose SUB_GRAB_SPEC_DEPOSIT_TRANSITION = new Pose(26.5, 25.5, 0);
    public static Pose TELEOP_SPEC_HANG_TRANSITION = new Pose(26, 60, 0);

    public static Pose TELEOP_SPEC_HANG_TRANSITION_FINAL = new Pose(35, 60, 0);

    public static Pose TELEOP_SPEC_PICKUP = new Pose();
}
