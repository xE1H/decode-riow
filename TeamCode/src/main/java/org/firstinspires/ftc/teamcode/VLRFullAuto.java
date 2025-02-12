package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Points.BUCKET_HIGH_SCORE_POSE;
import static org.firstinspires.ftc.teamcode.Points.START_POSE;
import static org.firstinspires.ftc.teamcode.helpers.pedro.PoseToPath.bezierPath;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.outoftheboxrobotics.photoncore.Photon;
import com.pedropathing.commands.FollowPath;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.ScheduleNext;
import org.firstinspires.ftc.teamcode.helpers.autoconfig.AutoConfigurator;
import org.firstinspires.ftc.teamcode.helpers.enums.Alliance;
import org.firstinspires.ftc.teamcode.helpers.opmode.VLRLinearOpMode;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.hang.HangSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.vision.Vision;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@TeleOp(name = "VLRFullAuto", group = "!TELEOP")
@Photon
public class VLRFullAuto extends VLRLinearOpMode {
    Alliance alliance = Alliance.BLUE;

    CommandScheduler cs;

    Follower f;
    Pose startPose = new Pose(10, 111.5, 0);

    /**
     * @noinspection unchecked
     */
    @Override
    public void run() {
        Constants.setConstants(FConstants.class, LConstants.class);

        cs = CommandScheduler.getInstance();
        GameStateController gameStateController = new GameStateController(this::getTimeSinceInit, true); // FORCING TELEOP FOR TESTING
        StrategyController strategyController = new StrategyController(gameStateController);

        VLRSubsystem.requireSubsystems(ArmSlideSubsystem.class, ArmRotatorSubsystem.class, ClawSubsystem.class, HangSubsystem.class, Vision.class);
        VLRSubsystem.initializeAll(hardwareMap);

        f = new Follower(hardwareMap);
        f.setStartingPose(startPose);

//        AutoConfigurator configurator = new AutoConfigurator(telemetry, gamepad1);
//        AutoConfigurator.Choice choice = configurator.multipleChoice("Select alliance:",
//                new AutoConfigurator.Choice("Red"),
//                new AutoConfigurator.Choice("Blue"));
//
//        alliance = choice.text.equals("Red") ? Alliance.RED : Alliance.BLUE;
//
//        configurator.review("Alliance: " + alliance);

        waitForStart();
        gameStateController.postInit();


        FollowPath toStartingPosition = new FollowPath(f, bezierPath(START_POSE, BUCKET_HIGH_SCORE_POSE)
                .setLinearHeadingInterpolation(START_POSE.getHeading(), BUCKET_HIGH_SCORE_POSE.getHeading())
                .build());

        cs.schedule(new SequentialCommandGroup(
                toStartingPosition,
                new ScheduleNext(strategyController, f, alliance)
        ));

        while (opModeIsActive()) {
            f.update();
        }
    }
}
