package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.pedroCommands.FollowPath;
import org.firstinspires.ftc.teamcode.auto.pedroCommands.HoldPoint;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.pathGeneration.Point;

import org.firstinspires.ftc.teamcode.helpers.commands.GrabBucketSample;
import org.firstinspires.ftc.teamcode.helpers.commands.ScoreBucketSample;
import org.firstinspires.ftc.teamcode.helpers.opmode.VLRLinearOpMode;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmState;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.MoveArmToDeposit;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.MoveArmInToRobot;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.MoveArmToIntake;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.SetSlideExtension;
import org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawAngle;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawState;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawTwist;

@Config
@Photon
@Autonomous(name = "VLRAuto_bucket")
public class VLRAuto_bucket extends VLRLinearOpMode {
    private Follower follower;

    private final double xStart = 10;
    private final double yStart = 111.5;

    public static double grab2 = 5;
    public static double grab3 = 28;
    private Point scorePoint = new Point(22, 123);
    private double scoreHeading = -50;

    @Override
    public void run() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        VLRSubsystem.requireSubsystems(ArmSlideSubsystem.class, ArmRotatorSubsystem.class, ClawSubsystem.class);
        VLRSubsystem.initializeAll(hardwareMap);

        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(xStart, yStart, 0));
        follower.setMaxPower(0.6);

        FollowPath.setStartingPoint(new Point(xStart, yStart));
        FollowPath.setFollower(follower);
        HoldPoint.setFollower(follower);

        waitForStart();
        schedulePath();
        ArmState.State prevState = ArmState.get();

        while (opModeIsActive()) {
//            follower.telemetryDebug(FtcDashboard.getInstance().getTelemetry());
        }
    }

    private void schedulePath() {

        CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
                new SetClawTwist(ClawConfiguration.TargetTwist.NORMAL),
                // to bucket
                new FollowPath(0, scoreHeading, scorePoint),
                new ScoreBucketSample(),

                new FollowPath(scoreHeading, -13, new Point(22, 125)),
                new GrabBucketSample(),

                new FollowPath(-13, scoreHeading, scorePoint),
                new ScoreBucketSample(),

                new FollowPath(scoreHeading, grab2, new Point(22, 125)),
                new GrabBucketSample(),

                new FollowPath(grab2, scoreHeading, scorePoint),
                new ScoreBucketSample(),

                new FollowPath(scoreHeading, grab3, new Point(25, 125)),
                new GrabBucketSample(),

                new FollowPath(grab3, scoreHeading, scorePoint),
                new ScoreBucketSample()


//                new FollowPath(-50, 13.8, new Point(22, 123)),
//                new ParallelRaceGroup(
//                        new HoldPoint(
//                                new Pose(22, 123, -50),
//                                true
//                        ),
//                        new SequentialCommandGroup(
//                                new MoveArmToDeposit(),
//                                new WaitCommand(100),
//                                new MoveArmInToRobot(),
//                                new WaitCommand(100)
//                        )
//                )


//                new HoldPoint(
//                        new Pose(22, 123, 13.8)
//                ),
//                new MoveArmToIntake(0.6),
//                new WaitCommand(600),
//                new SetClawAngle(ClawConfiguration.TargetAngle.DOWN),
//                new WaitCommand(200)

//                new SetClawState(ClawConfiguration.TargetState.CLOSED_NORMAL),
//                new WaitCommand(100),
//                new SetClawAngle(ClawConfiguration.TargetAngle.UP),
//                new WaitCommand(50),
//                new MoveArmInToRobot(),
//                new FollowPath(13.8, -50, new Point(22, 123)),
//                new MoveArmToDeposit(),
//                new WaitCommand(100),
//                new MoveArmInToRobot()
        ));
    }

}
