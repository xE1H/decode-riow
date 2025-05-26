package org.firstinspires.ftc.teamcode.auto.specimen;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.MathFunctions;

import org.firstinspires.ftc.teamcode.helpers.pedro.MathUtils;

@Config
public class PointsSpecimen {
    public static double OFFSET = 8.5;

    public static Pose START_POSE = new Pose(10, 57 + OFFSET, Math.toRadians(0)); //63.5

    public static Pose SCORE_PRELOAD_AND_SUB_PICKUP = new Pose(40.1, 57 + OFFSET, Math.toRadians(0));

    public static Pose PICK_UP_SAMPLE_1 = new Pose(29, 14.9 + OFFSET, Math.toRadians(0));

    public static Pose PICK_UP_SAMPLE_2 = new Pose(29, 5.1 + OFFSET, Math.toRadians(0));

    public static Pose PICK_UP_SAMPLE_3 = new Pose(29, 3.2 + OFFSET, Math.toRadians(-22));

    public static Pose DEPOSIT_SAMPLE_3_START = new Pose(29, 16 + OFFSET, Math.toRadians(0));

    public static Pose DEPOSIT_SAMPLE_3_END = new Pose(24.4, 16 + OFFSET, Math.toRadians(0));

    public static Pose SCORE_SECOND_SPECIMEN = new Pose(34.25, 53 + OFFSET, Math.toRadians(0));

    public static Pose SECOND_SPECIMEN_CONTROL = new Pose(23.9, 53.16 + OFFSET, Math.toRadians(0));

    public static Pose DRIVE_BACK = new Pose(28, SCORE_SECOND_SPECIMEN.getY(), Math.toRadians(0));

    public static Pose PICK_UP_SPECIMENS_FROM_HUMAN_PLAYER = new Pose(22, 40 + OFFSET, Math.toRadians(-120));

    private static final double distance = 13; //inches to go straight before pickup
    public static final double pickupAngle =  MathUtils.normalizeAngleRads(PICK_UP_SPECIMENS_FROM_HUMAN_PLAYER.getHeading()) - Math.toRadians(90);

    public static Pose MIDPOINT_BEFORE_PICKUP = new Pose(
            PICK_UP_SPECIMENS_FROM_HUMAN_PLAYER.getX() + distance * Math.sin(pickupAngle),
            PICK_UP_SPECIMENS_FROM_HUMAN_PLAYER.getY() + distance * Math.cos(pickupAngle),
            PICK_UP_SPECIMENS_FROM_HUMAN_PLAYER.getHeading());

    public static Pose SCORE_SPECIMEN_BACK = new Pose(26.5, 61 + OFFSET, Math.toRadians(-180)); //x = 31.55

    public static double DELTA = 1.75; //Y OFFSET FOR 4TH AND 5TH SPECIMEN

    public static Pose SUB_GRAB_SPEC = new Pose(64, 24.5, Math.toRadians(90));
    public static Pose SUB_GRAB_SPEC_CONTROL = new Pose(27, 20, Math.toRadians(0));

    public static Pose SUB_GRAB_SPEC_DEPOSIT = new Pose(25, 19.5, 0);

    public static Pose SUB_GRAB_SPEC_DEPOSIT_TRANSITION_CONTROL = new Pose(30, 25.5, 0);
    public static Pose SUB_GRAB_SPEC_DEPOSIT_TRANSITION = new Pose(26.5, 25.5, 0);
    public static Pose TELEOP_SPEC_HANG_TRANSITION = new Pose(26, 60, 0);

    public static Pose TELEOP_SPEC_HANG_TRANSITION_FINAL_FWD = new Pose(35, 60, 0);

    public static Pose TELEOP_SPEC_HANG_FINAL_BACK = new Pose(31, 62.5, Math.toRadians(-180));
}
