package org.firstinspires.ftc.teamcode.auto.sample;

import static org.firstinspires.ftc.teamcode.auto.sample.PointsSample.BUCKET_HIGH_SCORE_POSE;
import static org.firstinspires.ftc.teamcode.auto.sample.PointsSample.BUCKET_HIGH_SCORE_POSE_SUB;
import static org.firstinspires.ftc.teamcode.auto.sample.PointsSample.DELTA;
import static org.firstinspires.ftc.teamcode.auto.sample.PointsSample.FIRST_MARK_GRAB;
import static org.firstinspires.ftc.teamcode.auto.sample.PointsSample.GRAB_POSES;
import static org.firstinspires.ftc.teamcode.auto.sample.PointsSample.PRELOAD_BUCKET_HIGH_SCORE_POSE;
import static org.firstinspires.ftc.teamcode.auto.sample.PointsSample.START_POSE;
import static org.firstinspires.ftc.teamcode.auto.sample.PointsSample.SUB_GRAB;
import static org.firstinspires.ftc.teamcode.auto.sample.PointsSample.SUB_GRAB_0;
import static org.firstinspires.ftc.teamcode.auto.sample.PointsSample.SUB_GRAB_CONTROL_1;
import static org.firstinspires.ftc.teamcode.auto.sample.PointsSample.SUB_GRAB_CONTROL_2;
import static org.firstinspires.ftc.teamcode.helpers.pedro.PoseToPath.bezierPath;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.pedropathing.commands.FollowPath;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.helpers.commands.CustomConditionalCommand;
import org.firstinspires.ftc.teamcode.helpers.commands.LogCommand;
import org.firstinspires.ftc.teamcode.helpers.commands.ScheduleRuntimeCommand;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.helpers.utils.GlobalTimer;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmState;
import org.firstinspires.ftc.teamcode.subsystems.arm.MainArmConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.arm.ResetRotator;
import org.firstinspires.ftc.teamcode.subsystems.arm.ResetSlides;
import org.firstinspires.ftc.teamcode.subsystems.arm.SetArmPosition;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawAngle;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawState;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawTwist;
import org.firstinspires.ftc.teamcode.subsystems.limelight.LimelightYoloReader;

import java.util.logging.Level;

public class AutonomousPeriodActionSample extends SequentialCommandGroup {
    private boolean sampleScored = false;
    private final LimelightYoloReader reader;
    private final double jointPathTValue = 0.94;

    public AutonomousPeriodActionSample(Follower follower, LimelightYoloReader reader) {
        this.reader = reader;

        addCommands(
                new SetArmPosition().setArmState(ArmState.State.IN_ROBOT),
                new SetClawAngle(ClawConfiguration.VerticalRotation.UP),
                new SetClawTwist(ClawConfiguration.HorizontalRotation.NORMAL),
                new SetClawState(ClawConfiguration.GripperState.CLOSED),

                //SCORE PRELOAD AND PICK UP FIRST SAMPLE FROM SPIKE MARK
                scorePreload(follower),

                //GRAB FIRST SPIKE MARK, DRIVE TO BUCKET, DRIVE TO SECOND
                grabAndScoreSpikeMarkSample(follower, 2),

                //GRAB SECOND SPIKE MARK, DRIVE TO BUCKET, DRIVE TO THIRD
                grabAndScoreSpikeMarkSample(follower, 3),

                //GRAB THIRD SPIKE MARK, DRIVE TO BUCKET, DRIVE TO SUB GRAB POSE
                grabAndScoreSpikeMarkSample(follower, 4),

                subCycle(follower, 5),

                subCycle(follower, 6),
                subCycle(follower, 7)
        );
    }


    private Command scorePreload(Follower follower) {
        return new ParallelCommandGroup(
                new SequentialCommandGroup(
                        new SetArmPosition().scoreSample(MainArmConfiguration.SAMPLE_SCORE_HEIGHT.HIGH_BASKET),
                        new WaitCommand(40),
                        //new WaitUntilCommand(() -> follower.atPose(PRELOAD_BUCKET_HIGH_SCORE_POSE, 3, 3, Math.toRadians(4))),
                        new InstantCommand(() -> sampleScored = true),
                        new SetArmPosition().intakeSample(0.32)
                ),

                new SequentialCommandGroup(
                        new FollowPath(follower, bezierPath(START_POSE, PRELOAD_BUCKET_HIGH_SCORE_POSE)
                                .setLinearHeadingInterpolation(START_POSE.getHeading(), PRELOAD_BUCKET_HIGH_SCORE_POSE.getHeading()).build()
                        ),
                        new WaitUntilCommand(() -> sampleScored),
                        new InstantCommand(() -> sampleScored = false),

                        new WaitCommand(200),
                        new FollowPath(follower, bezierPath(PRELOAD_BUCKET_HIGH_SCORE_POSE, FIRST_MARK_GRAB)
                                .setLinearHeadingInterpolation(PRELOAD_BUCKET_HIGH_SCORE_POSE.getHeading(), FIRST_MARK_GRAB.getHeading()).build()
                        )
                )
        );
    }


    private Command subCycle(Follower follower, int sample) {
        double dx_first = (sample - 5) * DELTA;
        double dx_second = (sample - 4) * DELTA;


        return new SequentialCommandGroup(
                new WaitCommand(50),
                new SetArmPosition().setArmState(ArmState.State.IN_ROBOT),
                new SubmersibleGrabV2(follower, reader),

                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new SetArmPosition().retract(),
                                new ConditionalCommand(
                                        new LogCommand("AUTONOMOUS PERIOD ACTIONS", "CLAW SAMPLE PICKED UP SUCCESSFULLY"),
                                        new LogCommand("AUTONOMOUS PERIOD ACTIONS", Level.SEVERE, "CLAW SAMPLE PICKUP FAILED"),
                                        ()-> VLRSubsystem.getInstance(ClawSubsystem.class).isSamplePresent()
                                ),

                                new SetArmPosition().scoreSample(MainArmConfiguration.SAMPLE_SCORE_HEIGHT.HIGH_BASKET),
                                new WaitUntilCommand(() -> follower.atPose(BUCKET_HIGH_SCORE_POSE_SUB, 3, 3, Math.toRadians(2))),
                                new InstantCommand(() -> sampleScored = true),

                                new ConditionalCommand(
                                        new SetArmPosition().retract(),

                                        ///dont retract ofter 7th sample cause no time bruh
                                        new SequentialCommandGroup(
                                                new SetClawState(ClawConfiguration.GripperState.OPEN),
                                                new WaitCommand(130),
                                                new SetClawAngle(ClawConfiguration.VerticalRotation.DOWN),
                                                new WaitCommand(50),
                                                new SetArmPosition().extensionAndAngleDegrees(0.8, 111)
                                        ),
                                        ()-> sample <= 6
                                )
                        ),

                        new SequentialCommandGroup(
                                new WaitCommand(100),
                                new FollowPath(follower, bezierPath(offsetX(dx_first, SUB_GRAB, SUB_GRAB_CONTROL_2, SUB_GRAB_CONTROL_1, SUB_GRAB_0))
                                        .setTangentHeadingInterpolation().setReversed(true).setPathEndTValueConstraint(jointPathTValue).build(), false),
                                new LogCommand("Auto bombo", "Passed 1st path"),
                                new FollowPath(follower, bezierPath(offsetX(dx_first, SUB_GRAB_0), BUCKET_HIGH_SCORE_POSE_SUB).setZeroPowerAccelerationMultiplier(1.75)
                                        .setConstantHeadingInterpolation(BUCKET_HIGH_SCORE_POSE.getHeading()).build()),
                                new LogCommand("Auto bombo", "Passed 2nd path"),

                                new WaitUntilCommand(() -> sampleScored),
                                new InstantCommand(() -> sampleScored = false),
                                new WaitCommand(200),

                                new CustomConditionalCommand(
                                        new SequentialCommandGroup(
                                                new FollowPath(follower, bezierPath(BUCKET_HIGH_SCORE_POSE_SUB, offsetX(dx_second, SUB_GRAB_0))
                                                        .setConstantHeadingInterpolation(SUB_GRAB_0.getHeading()).setPathEndTValueConstraint(jointPathTValue).build(), false),
                                                new FollowPath(follower, bezierPath(offsetX(dx_second, SUB_GRAB_0, SUB_GRAB_CONTROL_1, SUB_GRAB_CONTROL_2, SUB_GRAB))
                                                        .setTangentHeadingInterpolation().build())
                                        ),

                                        () -> sample <= 6
                                )
                        )
                ),
                GlobalTimer.logTime("TIME AFTER SCORING " + sample + "th SAMPLE: ")
        );
    }

    private Pose offsetX(double xOffset, Pose pose){
        return new Pose(pose.getX() + xOffset, pose.getY(), pose.getHeading());
    }

    private Pose[] offsetX(double xOffset, Pose... poses){
        for (int i = 0; i < poses.length; i++){
            poses[i] = offsetX(xOffset, poses[i]);
        }
        return poses;
    }


    private Command grabAndScoreSpikeMarkSample(Follower follower, int sample) {
        return new ParallelCommandGroup(
                new SequentialCommandGroup(
                        new SetArmPosition().scoreSample(MainArmConfiguration.SAMPLE_SCORE_HEIGHT.HIGH_BASKET),

                        new WaitUntilCommand(() -> follower.atPose(BUCKET_HIGH_SCORE_POSE, 4, 4, Math.toRadians(5))),
                        new InstantCommand(() -> sampleScored = true),
                        new SetArmPosition().setArmState(ArmState.State.SAMPLE_SCORE),

                        new ConditionalCommand(
                                new SetArmPosition().intakeSampleAuto(sample == 3 ? 0.29 : 0.3285, sample == 3 ? 0.685 : 0.5),
                                new SetArmPosition().retract().andThen(new SetClawState(ClawConfiguration.GripperState.OPEN).andThen(
                                        new ParallelCommandGroup(
                                                new ResetSlides(),
                                                new ResetRotator()
                                        ))),
                                () -> sample <= 3
                        )
                ),


                new SequentialCommandGroup(
                        new WaitCommand(100),
                        new FollowPath(follower, bezierPath(GRAB_POSES[sample - 2], BUCKET_HIGH_SCORE_POSE)
                                .setLinearHeadingInterpolation(GRAB_POSES[sample - 2].getHeading(), BUCKET_HIGH_SCORE_POSE.getHeading()).build()),

                        new WaitUntilCommand(() -> sampleScored),
                        new InstantCommand(() -> sampleScored = false),

                        new WaitCommand(100),
                        new ConditionalCommand(
                                new FollowPath(follower, bezierPath(BUCKET_HIGH_SCORE_POSE, sample != 4 ? GRAB_POSES[sample - 1] : new Pose())
                                        .setLinearHeadingInterpolation(BUCKET_HIGH_SCORE_POSE.getHeading(), sample != 4 ? GRAB_POSES[sample - 1].getHeading() : new Pose().getHeading()).build()),

                                new SequentialCommandGroup(
                                        new FollowPath(follower, bezierPath(BUCKET_HIGH_SCORE_POSE, SUB_GRAB_0)
                                                .setConstantHeadingInterpolation(SUB_GRAB_0.getHeading()).setPathEndTValueConstraint(jointPathTValue).build(), false),
                                        new FollowPath(follower, bezierPath(SUB_GRAB_0, SUB_GRAB_CONTROL_1, SUB_GRAB_CONTROL_2, SUB_GRAB)
                                                .setTangentHeadingInterpolation().build())
                                ),

                                () -> sample <= 3
                        )

                )
        );
    }
}