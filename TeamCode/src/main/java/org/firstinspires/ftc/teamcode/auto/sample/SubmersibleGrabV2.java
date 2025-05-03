package org.firstinspires.ftc.teamcode.auto.sample;

import static org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideConfiguration.MAX_POSITION;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.commands.HoldPoint;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;

import org.firstinspires.ftc.teamcode.helpers.commands.InstantCommand;
import org.firstinspires.ftc.teamcode.helpers.controls.rumble.RumbleControls;
import org.firstinspires.ftc.teamcode.helpers.enums.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.arm.SetArmPosition;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawAngle;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawState;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawTwist;
import org.firstinspires.ftc.teamcode.subsystems.limelight.LimelightYoloReader;
import org.firstinspires.ftc.teamcode.subsystems.limelight.commands.RequestLimelightFrame;
import org.firstinspires.ftc.teamcode.subsystems.limelight.commands.WaitUntilNextLimelightFrame;

import java.util.logging.Logger;

@Config
public class SubmersibleGrabV2 extends SequentialCommandGroup {
    public static double DISTANCE_TO_ROBOT_FRONT = (224.5 - 6.6) / 24.5;

    public static double SUBMERSIBLE_BOUNDS_X1 = 0.5;
    public static double SUBMERSIBLE_BOUNDS_Y1 = 0.5;

    public static double SUBMERSIBLE_BOUNDS_X2 = 0.5;
    public static double SUBMERSIBLE_BOUNDS_Y2 = 0.5;

    private final Logger logger = Logger.getLogger("SubmersibleGrabV2");


    private final SequentialCommandGroup submersibleGrabCommand = new SequentialCommandGroup();

    double sampleAngle = 90;

    public SubmersibleGrabV2(Follower f, LimelightYoloReader reader, RumbleControls rc) {
        addCommands(
                new SetClawState(ClawConfiguration.GripperState.OPEN),
                new SetClawAngle(ClawConfiguration.VerticalRotation.UP),
                new SetClawTwist(ClawConfiguration.HorizontalRotation.NORMAL),


                new RequestLimelightFrame(reader),
                new WaitUntilNextLimelightFrame(reader),


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
                                if (rc != null) rc.doubleBlip();
                                return;
                            }
                            if (sampleAngle < 0) sampleAngle += 180;

                            logger.info("Going for sample: " + sample.getColor() + " in X: " + sample.getX() + ", Y: " + sample.getY() + ", angle: " + sampleAngle);

                            generateSubmersibleGrabCommand(f, sample);
                        }
                    }
                },
                submersibleGrabCommand
        );

    }

    private void generateSubmersibleGrabCommand(Follower f, LimelightYoloReader.Limelight.Sample sample) {
        logger.info("Current pose X: " + f.getPose().getX() + ", Y: " + f.getPose().getY() + ", heading: " + f.getPose().getHeading());
        double angle = f.getPose().getHeading() - Math.PI / 2; // Offset orientation by 90 deg to not mess up the distances

        double xAbs = -(f.getPose().getX() - sample.getX() * Math.cos(angle) + (sample.getY() + DISTANCE_TO_ROBOT_FRONT) * Math.sin(angle));
        double yAbs = -(f.getPose().getY() - sample.getX() * Math.sin(angle) - (sample.getY() + DISTANCE_TO_ROBOT_FRONT) * Math.cos(angle));
        logger.info("Absolute coordinates X: " + xAbs + ", Y: " + yAbs);

        boolean isInSubmersible = xAbs > SUBMERSIBLE_BOUNDS_X1 && xAbs < SUBMERSIBLE_BOUNDS_X2 && yAbs > SUBMERSIBLE_BOUNDS_Y1 && yAbs < SUBMERSIBLE_BOUNDS_Y2;
        if (isInSubmersible) {
            logger.info("Sample is in submersible bounds");
            // todo
        } else {
            logger.info("Sample is not in submersible bounds");
            // Vector to sample
            double vectorX = xAbs - f.getPose().getX();
            double vectorY = yAbs - f.getPose().getY();
            logger.info("Vector to sample X: " + vectorX + ", Y: " + vectorY);
            double vectorAngle = Math.atan2(vectorY, vectorX);
            double vectorLen = Math.sqrt(Math.pow(vectorX, 2) + Math.pow(vectorY, 2));
            logger.info("Vector angle: " + vectorAngle);
            logger.info("Vector length: " + vectorLen);

            double strafeLength = -(vectorX * Math.cos(angle) + vectorY * Math.sin(angle));
            double forwardLength = -(vectorX * Math.sin(angle) - vectorY * Math.cos(angle)) - DISTANCE_TO_ROBOT_FRONT;
            logger.info("Strafe length: " + strafeLength + ", Slide extension length: " + forwardLength);

            Pose currentPose = f.getPose();
            currentPose.setHeading(vectorAngle);
            submersibleGrabCommand.addCommands(
                    new ParallelCommandGroup(
                            new InstantCommand() {
                                @Override
                                public void run() {
                                    f.turnTo(Math.toDegrees(vectorAngle));
                                }
                            },

//                            new HoldPoint(f, currentPose).withTimeout(1000),
                            new SetArmPosition().intakeSampleAuto((0.7742 * (vectorLen - DISTANCE_TO_ROBOT_FRONT + 1.5)) / MAX_POSITION, ((sampleAngle - f.getPose().getHeading() + vectorAngle) / -180.0) + 1)
                    )
            );
        }

//        submersibleGrabCommand.addCommands(
//                new ParallelCommandGroup(
//                        new MoveRelative(f, -sample.getX(), 0),
//                        new SetArmPosition().intakeSampleAuto((0.7742 * (sample.getY() + 1.5)) / MAX_POSITION, (angle / -180.0) + 1)
//
//                )
//        );
    }


    public SubmersibleGrabV2(Follower f, LimelightYoloReader reader) {
        this(f, reader, null);
    }
}
