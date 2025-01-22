package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.pedroCommands.FollowPath;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.helpers.opmode.VLRLinearOpMode;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.helpers.utils.GlobalConfig;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmState;
import org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.chassis.Chassis;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.vision.Point3d;
import org.firstinspires.ftc.teamcode.subsystems.vision.SampleProcessor;
import org.firstinspires.ftc.teamcode.subsystems.vision.Vision;

import java.util.List;

@Autonomous
public class VisionAuto extends VLRLinearOpMode {
    @Override
    public void run() {
        VLRSubsystem.requireSubsystems(Chassis.class, ArmRotatorSubsystem.class, ArmSlideSubsystem.class, ClawSubsystem.class, Vision.class);
        VLRSubsystem.initializeAll(hardwareMap);

        Follower follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));
        follower.setMaxPower(0.6);

        FollowPath.setStartingPoint(new Point(0, 0));
        FollowPath.setFollower(follower);

        waitForStart();

        Vision vision = VLRSubsystem.getInstance(Vision.class);
        ArmSlideSubsystem ass = VLRSubsystem.getInstance(ArmSlideSubsystem.class);

        while (opModeIsActive()) {
            List<SampleProcessor.Detection> results = vision.getDetectionResults();
            // Pick closest result
            SampleProcessor.Detection closest = null;
            for (SampleProcessor.Detection result : results) {
                if (closest == null || getDistance(result.getWorldPos()) < getDistance(closest.getWorldPos())) {
                    closest = result;
                }
            }
            if (closest != null) {
                ass.setTargetPosition(ArmSlideConfiguration.TICKS_PER_IN * closest.getWorldPos().y);
                sleep(10000);
            }
        }
    }

    public double getDistance(Point3d point) {
        // Calculate distance from 0

        return Math.sqrt(Math.pow(point.x, 2) + Math.pow(point.y, 2) + Math.pow(point.z, 2));
    }
}
