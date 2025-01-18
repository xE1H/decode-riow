package org.firstinspires.ftc.teamcode.auto.pedroPathing.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.helpers.opmode.VLRLinearOpMode;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.helpers.utils.GlobalConfig;
import org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;
import org.opencv.core.Mat;

/**
 * This is the StraightBackAndForth autonomous OpMode. It runs the robot in a specified distance
 * straight forward. On reaching the end of the forward Path, the robot runs the backward Path the
 * same distance back to the start. Rinse and repeat! This is good for testing a variety of Vectors,
 * like the drive Vector, the translational Vector, and the heading Vector. Remember to test your
 * tunings on CurvedBackAndForth as well, since tunings that work well for straight lines might
 * have issues going in curves.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/12/2024
 */
@Config
@Photon
@Autonomous(name = "Straight Back And Forth", group = "Autonomous Pathing Tuning")
public class StraightBackAndForth extends VLRLinearOpMode {
    private Telemetry telemetryA;

    public static double DISTANCE = 10;
    public static int ANGLE = 0;
    private boolean forward = true;
    public static boolean inverted = true;
    public static double translationalConstraint = 0.2;

    private Follower follower;

    private Path forwards;
    private Path backwards;


    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the FTC Dashboard.
     */
    @Override
    public void run() {
        VLRSubsystem.requireSubsystems(ArmSlideSubsystem.class, ArmRotatorSubsystem.class, ClawSubsystem.class);
        VLRSubsystem.initializeAll(hardwareMap);
        GlobalConfig.setIsInvertedMotors(inverted);

        follower = new Follower(hardwareMap);
        follower.setMaxPower(0.6);
        follower.setStartingPose(new Pose(0, 0, Math.toRadians(ANGLE)));
        forwards = new Path(new BezierLine(new Point(0, 0, Point.CARTESIAN), new Point(DISTANCE, 0, Point.CARTESIAN)));
        forwards.setConstantHeadingInterpolation(Math.toRadians(ANGLE));
//        forwards.set
        backwards = new Path(new BezierLine(new Point(DISTANCE, 0, Point.CARTESIAN), new Point(0, 0, Point.CARTESIAN)));
        backwards.setConstantHeadingInterpolation(Math.toRadians(ANGLE));
        backwards.setPathEndTranslationalConstraint(translationalConstraint);
        follower.followPath(forwards);

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addLine("This will run the robot in a straight line going " + DISTANCE
                + " inches forward. The robot will go forward and backward continuously"
                + " along the path. Make sure you have enough room.");
        telemetryA.update();
        waitForStart();
        while (opModeIsActive()) {

            follower.update();
            if (!follower.isBusy()) {
                if (forward) {
                    forward = false;
                    follower.followPath(backwards);
                } else {
                    forward = true;
                    follower.followPath(forwards);
                }
            }

//            telemetryA.addData("going forward", forward);
//            follower.telemetryDebug(telemetryA);
        }
    }
}
