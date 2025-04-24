package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Pose;

@Config
public class Points_specimen {

    public static Pose START_POSE = new Pose(10, 57, Math.toRadians(0));

    public static Pose SCORE_PRELOAD_AND_SUB_PICKUP = new Pose(35.65, 57, Math.toRadians(0));

    public static Pose PICK_UP_SAMPLE_1 = new Pose(29, 14.5, Math.toRadians(0));

    public static Pose PICK_UP_SAMPLE_2 = new Pose(29, 4.5, Math.toRadians(0));

    public static Pose PICK_UP_SAMPLE_3 = new Pose(29, 3, Math.toRadians(-21.75));

    public static Pose DEPOSIT_SAMPLE_3 = new Pose(24.8,15, Math.toRadians(0));

    public static Pose SCORE_SECOND_SPECIMEN = new Pose(35.65, 60, Math.toRadians(0));

    public static Pose DRIVE_BACK = new Pose(25, SCORE_SECOND_SPECIMEN.getY(), Math.toRadians(0));

    public static Pose PICK_UP_SPECIMENS_FROM_HUMAN_PLAYER = new Pose(22, 40, Math.toRadians(-120));

    public static Pose SCORE_SPECIMEN_BACK = new Pose(31.4, 63, Math.toRadians(-180));

    public static Pose SCORE_SPECIMEN_BACK_4 = new Pose(31.5, 67, Math.toRadians(-180));

    public static Pose SCORE_SPECIMEN_BACK_5 = new Pose(31.5, 71, Math.toRadians(-180));
}
