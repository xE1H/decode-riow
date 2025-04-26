package org.firstinspires.ftc.teamcode.auto.sample;

import static org.firstinspires.ftc.teamcode.auto.sample.Points_sample.*;
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

import org.firstinspires.ftc.teamcode.helpers.commands.CustomConditionalCommand;
import org.firstinspires.ftc.teamcode.subsystems.arm.MainArmConfiguration;
import org.firstinspires.ftc.teamcode.helpers.commands.ScheduleRuntimeCommand;
import org.firstinspires.ftc.teamcode.subsystems.arm.SetArmPosition;
import org.firstinspires.ftc.teamcode.subsystems.blinkin.SetPattern;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawAngle;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawState;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawTwist;

public class AutonomousPeriodActionSample extends SequentialCommandGroup {
    private boolean sampleScored = false;
    private int currentSample = 0;

    public AutonomousPeriodActionSample(Follower follower) {
        addCommands(
                new SetClawAngle(ClawConfiguration.VerticalRotation.UP),
                new SetClawTwist(ClawConfiguration.HorizontalRotation.NORMAL),
                new SetClawState(ClawConfiguration.GripperState.CLOSED),
                new SetPattern().rainbow(),

                //SCORE PRELOAD AND PICK UP FIRST SAMPLE FROM SPIKE MARK
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new SetArmPosition().scoreSample(MainArmConfiguration.SAMPLE_SCORE_HEIGHT.HIGH_BASKET),
                                new WaitUntilCommand(()-> follower.atPose(BUCKET_HIGH_SCORE_POSE, 4, 4, Math.toRadians(8))),
                                new InstantCommand(()-> sampleScored = true),
                                new SetArmPosition().retract().alongWith(new SetPattern().oceanPalette()),
                                new SetArmPosition().intakeSample(0.3).andThen(new SetPattern().rainbow())
                        ),

                        new SequentialCommandGroup(
                                new FollowPath(follower, bezierPath(START_POSE, BUCKET_HIGH_SCORE_POSE)
                                        .setLinearHeadingInterpolation(START_POSE.getHeading(), BUCKET_HIGH_SCORE_POSE.getHeading()).build()
                                ),
                                new WaitUntilCommand(()-> sampleScored),
                                new InstantCommand(()-> sampleScored = false),
                                new WaitCommand(150),
                                new FollowPath(follower, bezierPath(BUCKET_HIGH_SCORE_POSE, FIRST_MARK_GRAB)
                                        .setLinearHeadingInterpolation(BUCKET_HIGH_SCORE_POSE.getHeading(), FIRST_MARK_GRAB.getHeading()).build()
                                )
                        )

                ),

                //GRAB FIRST SPIKE MARK, DRIVE TO BUCKET, DRIVE TO SECOND
                grabAndScoreSpikeMarkSample(follower),

                //GRAB SECOND SPIKE MARK, DRIVE TO BUCKET, DRIVE TO THIRD
                grabAndScoreSpikeMarkSample(follower),

                //GRAB THIRD SPIKE MARK, DRIVE TO BUCKET, DRIVE TO SUB GRAB POSE
                grabAndScoreSpikeMarkSample(follower)
        );
    }



    private Command grabAndScoreSpikeMarkSample(Follower follower){
        return new ParallelCommandGroup(
                new SequentialCommandGroup(
                        new SetArmPosition().retract(),
                        new SetArmPosition().scoreSample(MainArmConfiguration.SAMPLE_SCORE_HEIGHT.HIGH_BASKET),
                        new WaitUntilCommand(()-> follower.atPose(BUCKET_HIGH_SCORE_POSE, 4, 4, Math.toRadians(5))),
                        new InstantCommand(()-> sampleScored = true),
                        new SetArmPosition().retract(),

                        new CustomConditionalCommand(
                                new SetArmPosition().intakeSample(0.3),
                                ()-> currentSample < 2
                        )
                ),


                new SequentialCommandGroup(
                        new WaitCommand(100),
                        new ScheduleRuntimeCommand(
                                ()-> new FollowPath(follower, bezierPath(GRAB_POSES[currentSample], BUCKET_HIGH_SCORE_POSE)
                                .setLinearHeadingInterpolation(GRAB_POSES[currentSample].getHeading(), BUCKET_HIGH_SCORE_POSE.getHeading()).build())
                        ),

                        new WaitUntilCommand(()-> sampleScored),
                        new InstantCommand(()-> sampleScored = false),
                        new WaitCommand(150),

                        new ConditionalCommand(
                                new SequentialCommandGroup(
                                        new InstantCommand(()-> currentSample ++),
                                        new ScheduleRuntimeCommand(
                                                ()-> new FollowPath(follower, bezierPath(BUCKET_HIGH_SCORE_POSE, GRAB_POSES[currentSample])
                                                .setLinearHeadingInterpolation(BUCKET_HIGH_SCORE_POSE.getHeading(), GRAB_POSES[currentSample].getHeading()).build())
                                        )
                                ),

                                new FollowPath(follower, bezierPath(BUCKET_HIGH_SCORE_POSE, SUB_GRAB)
                                        .setLinearHeadingInterpolation(BUCKET_HIGH_SCORE_POSE.getHeading(), SUB_GRAB.getHeading()).build()
                                ),

                                ()-> currentSample < 2
                        )
                )
        );
    }
}