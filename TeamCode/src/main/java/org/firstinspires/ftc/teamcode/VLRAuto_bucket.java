package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.auto.pedroCommands.FollowPath;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.helpers.opmode.VLRLinearOpMode;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;

@Config
@Photon
@Autonomous(name = "VLRAuto_bucket")
public class VLRAuto_bucket extends VLRLinearOpMode {
    private Follower follower;

    private final double xStart = 10;
    private final double yStart = 111.5;


    @Override
    public void run() {
        VLRSubsystem.requireSubsystems(ArmSlideSubsystem.class, ArmRotatorSubsystem.class, ClawSubsystem.class);
        VLRSubsystem.initializeAll(hardwareMap);

        ArmRotatorSubsystem arm = VLRSubsystem.getInstance(ArmRotatorSubsystem.class);
        ArmSlideSubsystem slides = VLRSubsystem.getInstance(ArmSlideSubsystem.class);

        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(xStart, yStart, 0));
        follower.setMaxPower(0.6);

        FollowPath.setStartingPoint(new Point(xStart, yStart));
        FollowPath.setFollower(follower);

        waitForStart();
        schedulePath();

        CommandScheduler.getInstance().run();

        while (opModeIsActive()) {
            telemetry.addData("current state", arm.getArmState());
            telemetry.addData("reachedTarget", slides.reachedTargetPosition());
            follower.telemetryDebug(FtcDashboard.getInstance().getTelemetry());
        }
    }


    private void schedulePath() {
        CommandScheduler.getInstance().schedule(new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new FollowPath(0, -50, new Point(23, 125))
//                        new SequentialCommandGroup(
//                                new WaitCommand(1000),
//                                new SetArmState_Deposit()
//                        )
                ),
                new WaitCommand(10000),
                //new SetArmState_InRobot(),
                new WaitCommand(10000)
        ));
    }

}
