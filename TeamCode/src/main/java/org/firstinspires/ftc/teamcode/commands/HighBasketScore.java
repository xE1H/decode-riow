package org.firstinspires.ftc.teamcode.commands;

import static org.firstinspires.ftc.teamcode.Points.BUCKET_HIGH_SCORE_POSE;
import static org.firstinspires.ftc.teamcode.Points.SUB_GRAB_POSE;
import static org.firstinspires.ftc.teamcode.Points.SUB_PRE_BEZIER_POSE;
import static org.firstinspires.ftc.teamcode.helpers.pedro.PoseToPath.bezierPath;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.commands.FollowPath;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.helpers.commands.InstantCommand;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmState;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.RetractArm;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.SetCurrentArmState;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.SetSlideExtension;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.sample.ScoreSample;
import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawAngle;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawTwist;

public class HighBasketScore extends SequentialCommandGroup {
    public HighBasketScore(Follower f) {
        addCommands(
                new ParallelCommandGroup(
                        new SetClawTwist(ClawConfiguration.HorizontalRotation.NORMAL),
                        new SetClawAngle(ClawConfiguration.VerticalRotation.UP),
                        new SetSlideExtension(ArmSlideConfiguration.TargetPosition.RETRACTED),
                        new FollowPath(f, bezierPath(SUB_PRE_BEZIER_POSE, BUCKET_HIGH_SCORE_POSE)
                                .setLinearHeadingInterpolation(SUB_GRAB_POSE.getHeading(), BUCKET_HIGH_SCORE_POSE.getHeading())
                                .build()), // drive to the high basket
                        new SequentialCommandGroup(
                                new SetCurrentArmState(ArmState.State.IN_ROBOT),
                                new WaitCommand(1600),
                                new ScoreSample(117),
                                new WaitCommand(100)
                        )
                )
        );
    }
}
