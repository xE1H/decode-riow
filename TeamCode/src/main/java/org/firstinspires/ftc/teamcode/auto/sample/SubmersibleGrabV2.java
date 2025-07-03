package org.firstinspires.ftc.teamcode.auto.sample;

import static org.firstinspires.ftc.teamcode.helpers.pedro.PoseToPath.bezierPath;
import static org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideConfiguration.MAX_POSITION;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.commands.FollowPath;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;

import org.firstinspires.ftc.teamcode.helpers.commands.CustomConditionalCommand;
import org.firstinspires.ftc.teamcode.helpers.commands.InstantCommand;
import org.firstinspires.ftc.teamcode.helpers.commands.LogCommand;
import org.firstinspires.ftc.teamcode.helpers.commands.ScheduleRuntimeCommand;
import org.firstinspires.ftc.teamcode.helpers.controls.rumble.RumbleControls;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmState;
import org.firstinspires.ftc.teamcode.subsystems.arm.SetArmPosition;
import org.firstinspires.ftc.teamcode.subsystems.blinkin.SetPattern;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawState;
import org.firstinspires.ftc.teamcode.subsystems.limelight.LimelightYoloReader;
import org.firstinspires.ftc.teamcode.subsystems.limelight.commands.RequestLimelightFrame;
import org.firstinspires.ftc.teamcode.subsystems.limelight.commands.WaitUntilNextLimelightFrame;

import java.util.logging.Logger;

@Config
public class SubmersibleGrabV2 extends SequentialCommandGroup {
    private final Logger logger = Logger.getLogger("SubmersibleGrabV2");

    private final LimelightYoloReader reader;
    private LimelightYoloReader.Limelight.Sample.Color sampleColour;

    private double positiveNegativeYLimit = 0;

    private final SequentialCommandGroup submersibleGrabCommand = new SequentialCommandGroup();
    double sampleAngle = 90;

    public SubmersibleGrabV2(Follower f, LimelightYoloReader reader, RumbleControls rc, boolean skipWaiting) {
        this.reader = reader;
        addCommands(
                new CustomConditionalCommand(
                        new SetClawState(ClawConfiguration.GripperState.OPEN),
                        ()-> ArmState.isCurrentState(ArmState.State.IN_ROBOT)
                ),

                new CustomConditionalCommand(
                        new RequestLimelightFrame(reader, f).andThen(new WaitUntilNextLimelightFrame(reader)),
                        ()-> !skipWaiting
                ),


                new InstantCommand() {
                    @Override
                    public void run() {
                        LimelightYoloReader.Limelight.Sample sample = reader.getBestSampleWithRetry();

                        if (sample == null) {
                            logger.warning("No best sample found");
                            if (rc != null) rc.doubleBlip();
                        } else {
                            sampleAngle = Math.toDegrees(sample.getAngle());
                            if (sampleAngle == -360) {
                                logger.warning("Best sample has no angle data");
                                if (rc != null) rc.singleBlip();
                                return;
                            }
                            if (sampleAngle < 0) sampleAngle += 180;

                            sampleColour = sample.getColor();
                            logger.info("Going for sample: " + sampleColour + " in X: " + sample.getX() + ", Y: " + sample.getY() + ", angle: " + sampleAngle);

                            generateSubmersibleGrabCommand(f, sample);
                        }
                    }
                },

                new ScheduleRuntimeCommand(()-> new SetPattern().blinkSampleColour(sampleColour)),
                submersibleGrabCommand
        );

    }

    private void generateSubmersibleGrabCommand(Follower f, LimelightYoloReader.Limelight.Sample sample) {
        logger.info("Current pose X: " + f.getPose().getX() + ", Y: " + f.getPose().getY() + ", heading: " + f.getPose().getHeading());

        Pose framePose = reader.getFollowerFramePose();
        Pose currentPose = f.getPose();

        if (framePose == null) framePose = currentPose.copy();

        double frameRobotHeading = framePose.getHeading();
        double currentRobotHeading = currentPose.getHeading();


        double sampleFieldX = framePose.getX() + sample.getX() * Math.cos(frameRobotHeading) - sample.getY() * Math.sin(frameRobotHeading);
        double sampleFieldY = framePose.getY() + sample.getX() * Math.sin(frameRobotHeading) + sample.getY() * Math.cos(frameRobotHeading);
        logger.info("Sample field position - X: " + sampleFieldX + ", Y: " + sampleFieldY);

        double dx = sampleFieldX - currentPose.getX();
        double dy = sampleFieldY - currentPose.getY();
        logger.info("Vector to sample - dx: " + dx + ", dy: " + dy);

        // Calculate strafe and arm extension component
        double strafeComponent = dx * Math.cos(currentRobotHeading) + dy * Math.sin(currentRobotHeading);
        double forwardComponent = -dx * Math.sin(currentRobotHeading) + dy * Math.cos(currentRobotHeading);
        logger.info("Strafe component: " + strafeComponent);
        logger.info("Forward component: " + forwardComponent);

        // Calculate the strafe destination
        double strafeX = currentPose.getX() + strafeComponent * Math.sin(currentRobotHeading);
        double strafeY = currentPose.getY() - strafeComponent * Math.cos(currentRobotHeading);

        logger.info("Strafe destination - X: " + strafeX + ", Y: " + strafeY);

        Pose strafePose = new Pose(strafeX, strafeY, currentRobotHeading);
        logger.info("CLAW TWIST: " + (sampleAngle / -180.0) + 1);

        if (positiveNegativeYLimit == 0 || strafeY > positiveNegativeYLimit) {
            submersibleGrabCommand.addCommands(
                    new ParallelCommandGroup(
                            new FollowPath(f, bezierPath(currentPose, strafePose)
                                    .setConstantHeadingInterpolation(currentRobotHeading).setPathEndTValueConstraint(0.95).setZeroPowerAccelerationMultiplier(5)
                                    .build())
                                    .withTimeout(600),

                            new SetArmPosition().intakeSampleAuto(
                                    (0.905 * (forwardComponent + 1.5)) / MAX_POSITION, //0.7742
                                    (sampleAngle / -180.0) + 1
                            )
                    ).andThen(new LogCommand("SKIBIDI LOGGER", "SKIBIDI SUB INTAKE FINITO"))
            );
        }
    }

    public SubmersibleGrabV2(Follower f, LimelightYoloReader reader, boolean skipWaiting) {
        this(f, reader, null, skipWaiting);
    }

    public SubmersibleGrabV2(Follower f, LimelightYoloReader reader, RumbleControls rc) {
        this(f, reader, rc, false);
    }

    public SubmersibleGrabV2(Follower f, LimelightYoloReader reader) {
        this(f, reader, null, false);
    }

    public SubmersibleGrabV2 setNegativeYLimit(double negativeYLimit){
        this.positiveNegativeYLimit = negativeYLimit;
        return this;
    }
}
