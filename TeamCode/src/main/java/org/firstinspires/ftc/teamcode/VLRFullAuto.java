package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Points.BUCKET_HIGH_SCORE_POSE;
import static org.firstinspires.ftc.teamcode.Points.START_POSE;
import static org.firstinspires.ftc.teamcode.helpers.pedro.PoseToPath.bezierPath;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.outoftheboxrobotics.photoncore.Photon;
import com.pedropathing.commands.FollowPath;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.helpers.autoconfig.AutoConfigurator;
import org.firstinspires.ftc.teamcode.helpers.enums.Alliance;
import org.firstinspires.ftc.teamcode.helpers.opmode.VLRLinearOpMode;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.hang.HangSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.vision.Vision;

@TeleOp(name = "VLRFullAuto", group = "!TELEOP")
@Photon
public class VLRFullAuto extends VLRLinearOpMode {
    Alliance alliance;

    CommandScheduler cs;

    Follower f;
    Pose startPose = new Pose(10, 111.5, 0);

    /**
     * @noinspection unchecked
     */
    @Override
    public void run() {
        cs = CommandScheduler.getInstance();
        GameStateController gameStateController = new GameStateController(this::getTimeSinceInit, true); // FORCING TELEOP FOR TESTING
        StrategyController strategyController = new StrategyController(gameStateController);

        VLRSubsystem.requireSubsystems(ArmSlideSubsystem.class, ArmRotatorSubsystem.class, ClawSubsystem.class, HangSubsystem.class, Vision.class);
        VLRSubsystem.initializeAll(hardwareMap);

        f = new Follower(hardwareMap);
        f.setStartingPose(startPose);

        AutoConfigurator configurator = new AutoConfigurator(telemetry, gamepad1);
        AutoConfigurator.Choice choice = configurator.multipleChoice("Select alliance:",
                new AutoConfigurator.Choice("Red"),
                new AutoConfigurator.Choice("Blue"));

        alliance = choice.text.equals("Red") ? Alliance.RED : Alliance.BLUE;

        configurator.review("Alliance: " + alliance);

        waitForStart();
        gameStateController.postInit();

        // simple version -- go to sub, take a sample, score in the high basket and do it 10 times

        FollowPath toStartingPosition = new FollowPath(f, bezierPath(START_POSE, BUCKET_HIGH_SCORE_POSE)
                .setLinearHeadingInterpolation(START_POSE.getHeading(), BUCKET_HIGH_SCORE_POSE.getHeading())
                .build());

        cs.schedule(toStartingPosition);

        while (opModeIsActive()) {
            f.update();
        }
    }
}
