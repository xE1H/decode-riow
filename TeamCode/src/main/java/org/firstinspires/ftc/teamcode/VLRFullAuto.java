package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Points.BUCKET_HIGH_SCORE_POSE;
import static org.firstinspires.ftc.teamcode.Points.BUCKET_HIGH_SCORE_TRUE_POSE;
import static org.firstinspires.ftc.teamcode.Points.START_POSE;
import static org.firstinspires.ftc.teamcode.Points.SUB_GRAB_POSE;
import static org.firstinspires.ftc.teamcode.Points.SUB_PRE_BEZIER_POSE;
import static org.firstinspires.ftc.teamcode.Points.SUB_PRE_PREGRAB_POSE;
import static org.firstinspires.ftc.teamcode.helpers.pedro.PoseToPath.bezierPath;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.outoftheboxrobotics.photoncore.Photon;
import com.pedropathing.commands.FollowPath;
import com.pedropathing.commands.HoldPoint;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.AutonomousPeriodActions;
import org.firstinspires.ftc.teamcode.commands.HighBasketScore;
import org.firstinspires.ftc.teamcode.commands.SubmersibleGrab;
import org.firstinspires.ftc.teamcode.helpers.autoconfig.AutoConfigurator;
import org.firstinspires.ftc.teamcode.helpers.commands.InstantCommand;
import org.firstinspires.ftc.teamcode.helpers.controls.rumble.RumbleControls;
import org.firstinspires.ftc.teamcode.helpers.enums.Alliance;
import org.firstinspires.ftc.teamcode.helpers.opmode.VLRLinearOpMode;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmState;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.ResetRotatorMotor;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.RetractArm;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.SetCurrentArmState;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.SetSlideExtension;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.sample.ScoreSample;
import org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.chassis.Chassis;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawAngle;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawTwist;
import org.firstinspires.ftc.teamcode.subsystems.hang.HangSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.hang.commands.SecondStageHangCommand;
import org.firstinspires.ftc.teamcode.subsystems.limelight.Limelight;
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
        GameStateController gameStateController = new GameStateController(this::getTimeSinceInit, false); // FORCING TELEOP FOR TESTING
        StrategyController strategyController = new StrategyController(gameStateController);

        VLRSubsystem.requireSubsystems(ArmSlideSubsystem.class, ArmRotatorSubsystem.class, ClawSubsystem.class, Limelight.class, Chassis.class);
        VLRSubsystem.initializeAll(hardwareMap);

        f = new Follower(hardwareMap);
        f.setStartingPose(startPose);

        AutoConfigurator configurator = new AutoConfigurator(telemetry, gamepad1);
        AutoConfigurator.Choice choice = configurator.multipleChoice("Select alliance:",
                new AutoConfigurator.Choice("Red"),
                new AutoConfigurator.Choice("Blue"));

        AutoConfigurator.Choice autoChoice = configurator.multipleChoice("Run auto:",
                new AutoConfigurator.Choice("Run"),
                new AutoConfigurator.Choice("Do not run"));


        alliance = choice.text.equals("Red") ? Alliance.RED : Alliance.BLUE;

        boolean runAuto = autoChoice.text.equals("Run");

        configurator.review("Alliance: " + alliance, "Auto: " + runAuto);

        waitForStart();
        VLRSubsystem.getInstance(Limelight.class).setAlliance(Alliance.BLUE);
        VLRSubsystem.getInstance(Limelight.class).enable();
        gameStateController.postInit();

        GamepadEx gp = new GamepadEx(gamepad1);
        RumbleControls rc = new RumbleControls(gamepad1);

        Chassis chassis = VLRSubsystem.getInstance(Chassis.class);

        if (runAuto) {
            cs.schedule(new SequentialCommandGroup(
                    new AutonomousPeriodActions(f),
                    new SequentialCommandGroup(
                            new ParallelCommandGroup(
                                    new RetractArm(),
                                    new SequentialCommandGroup(
                                            new WaitCommand(600),
                                            new org.firstinspires.ftc.teamcode.helpers.commands.InstantCommand() {
                                                @Override
                                                public void run() {
                                                    VLRSubsystem.getInstance(Limelight.class).enable();
                                                }
                                            },
                                            new FollowPath(f, bezierPath(BUCKET_HIGH_SCORE_POSE, SUB_PRE_BEZIER_POSE,
                                                    new Pose(SUB_PRE_PREGRAB_POSE.getX(), SUB_PRE_PREGRAB_POSE.getY(), SUB_PRE_PREGRAB_POSE.getHeading()),
                                                    new Pose(SUB_GRAB_POSE.getX(), SUB_GRAB_POSE.getY(), SUB_GRAB_POSE.getHeading()))
                                                    .setLinearHeadingInterpolation(BUCKET_HIGH_SCORE_POSE.getHeading(), SUB_GRAB_POSE.getHeading())
                                                    .build()).withTimeout(2500)
                                    )
                            ),
                            new WaitCommand(600),
                            new SubmersibleGrab(f, alliance, rc)
                    ),
                    new ParallelCommandGroup(
                            new SetClawTwist(ClawConfiguration.HorizontalRotation.NORMAL),
                            new SetClawAngle(ClawConfiguration.VerticalRotation.UP),
                            new SetSlideExtension(ArmSlideConfiguration.TargetPosition.RETRACTED),
                            new FollowPath(f, bezierPath(new Pose(SUB_GRAB_POSE.getX(), SUB_GRAB_POSE.getY(), SUB_GRAB_POSE.getHeading()), SUB_PRE_BEZIER_POSE, BUCKET_HIGH_SCORE_TRUE_POSE)
                                    .setLinearHeadingInterpolation(SUB_GRAB_POSE.getHeading(), BUCKET_HIGH_SCORE_POSE.getHeading())
                                    .build()), // drive to the high basket
                            new SequentialCommandGroup(
                                    new SetCurrentArmState(ArmState.State.IN_ROBOT),
                                    new WaitCommand(1600),
                                    new SequentialCommandGroup(
                                            new ScoreSample(110),
                                            new WaitCommand(300),
                                            new RetractArm()
                                    )
                            )
                    )
                    //new ScheduleNext(strategyController, f, alliance)
            ));
        }
        final boolean[] followerActive = {true};

        while (opModeIsActive()) {
            gp.readButtons();
            if (gp.wasJustPressed(GamepadKeys.Button.A)) {
                followerActive[0] = !followerActive[0];
                if (followerActive[0]) {
                    rc.doubleBlip();
                    f.holdPoint(f.getPose());
                } else {
                    rc.singleBlip();
                }
            }
            if (gp.wasJustPressed(GamepadKeys.Button.B)) {
                cs.schedule(new RetractArm());
            }

            if (gp.wasJustPressed(GamepadKeys.Button.X)) {
                VLRSubsystem.getInstance(ArmSlideSubsystem.class).setPowerOverride(true);
                VLRSubsystem.getInstance(ArmSlideSubsystem.class).setMotorPower(-0.3);
            } else if (gp.wasJustReleased(GamepadKeys.Button.X)) {
                VLRSubsystem.getInstance(ArmSlideSubsystem.class).setPowerOverride(false);
            }

            if (gp.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                cs.schedule(new SecondStageHangCommand(() -> gamepad1.dpad_up));
            }

            if (gp.wasJustPressed(GamepadKeys.Button.Y)) {
                cs.schedule(new ResetRotatorMotor());
            }

            if (gp.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                followerActive[0] = true;
                f.holdPoint(new Pose(f.getPose().getX(), f.getPose().getY(), SUB_GRAB_POSE.getHeading()));
                double headingError = Math.abs(f.getPose().getHeading() - SUB_GRAB_POSE.getHeading());
                cs.schedule(
                        new SequentialCommandGroup(
                                new WaitCommand((long) (headingError * 30)),
                                new SubmersibleGrab(f, alliance, rc)
                        ));
            }
            if (gp.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                followerActive[0] = true;
                cs.schedule(
                        new SequentialCommandGroup(
                                new HighBasketScore(f, new InstantCommand() {
                                    @Override
                                    public void run() {
                                        followerActive[0] = false;
                                        rc.singleBlip();
                                    }
                                })
                        )
                );
            }
            if (followerActive[0]) {
                f.update();
            } else {
                chassis.drive(gp.getLeftY(), -gp.getLeftX(), -gp.getRightX() * 0.3);
                f.updatePose();
            }
            //f.telemetryDebug(FtcDashboard.getInstance().getTelemetry());
        }
    }
}
