package org.firstinspires.ftc.teamcode.commands;

import static org.firstinspires.ftc.teamcode.Points.BUCKET_HIGH_SCORE_POSE;
import static org.firstinspires.ftc.teamcode.Points.SUB_GRAB_POSE;
import static org.firstinspires.ftc.teamcode.Points.SUB_PRE_BEZIER_POSE;
import static org.firstinspires.ftc.teamcode.helpers.pedro.PoseToPath.bezierPath;
import static org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideConfiguration.MAX_POSITION;
import static org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideConfiguration.TICKS_PER_IN;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.pedropathing.commands.FollowPath;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.helpers.commands.InstantCommand;
import org.firstinspires.ftc.teamcode.helpers.enums.Alliance;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.RetractArm;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.SetSlideExtension;
import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawAngle;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawState;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawTwist;
import org.firstinspires.ftc.teamcode.subsystems.vision.BestSampleDeterminer;
import org.firstinspires.ftc.teamcode.subsystems.vision.OrientationDeterminerPostProcessor;
import org.firstinspires.ftc.teamcode.subsystems.vision.Vision;
import org.firstinspires.ftc.teamcode.subsystems.vision.commands.ProcessFrame;

import java.util.List;

public class SubmersibleGrab extends SequentialCommandGroup {

    private SequentialCommandGroup submersibleGrabCommand;

    public SubmersibleGrab(Follower f, Alliance alliance) {
        // 1. Drive to the submersible
        // 2. run camera
        // 3. grab sample

        addCommands(
                new ParallelCommandGroup(
                        new RetractArm(),
                        new SequentialCommandGroup(
                                new WaitCommand(600),
                                new FollowPath(f, bezierPath(SUB_PRE_BEZIER_POSE, SUB_GRAB_POSE)
                                        .setLinearHeadingInterpolation(BUCKET_HIGH_SCORE_POSE.getHeading(), SUB_GRAB_POSE.getHeading())
                                        .build())
                        )
                ),
                new ParallelCommandGroup(
                        new ProcessFrame(),
                        new SequentialCommandGroup(
                                new WaitCommand(100),
                                new SetClawState(ClawConfiguration.GripperState.OPEN),
                                new SetClawAngle(ClawConfiguration.VerticalRotation.DEPOSIT),
                                new SetClawTwist(ClawConfiguration.HorizontalRotation.NORMAL)
                        )
                ),
                new InstantCommand() {
                    @Override
                    public void run() {
                        List<OrientationDeterminerPostProcessor.SampleOrientation> samples = VLRSubsystem.getInstance(Vision.class).getSampleOrientations();
                        OrientationDeterminerPostProcessor.SampleOrientation sample = BestSampleDeterminer.determineBestSample(samples, alliance);
                        System.out.println("Going for sample: " + sample.color + " in X: " + sample.relativeX + " Y: " + sample.relativeY);
                        generateSubmersibleGrabCommand(f, sample);
                    }
                },
                submersibleGrabCommand
        );

    }

    private void generateSubmersibleGrabCommand(Follower f, OrientationDeterminerPostProcessor.SampleOrientation sample) {
        submersibleGrabCommand.addCommands(
                new SetSlideExtension((TICKS_PER_IN * (sample.relativeY + 2)) / MAX_POSITION),
                new ParallelCommandGroup(
                        new MoveRelative(f, -sample.relativeX + 0.6, 0),
                        new WaitUntilCommand(VLRSubsystem.getInstance(ArmSlideSubsystem.class)::reachedTargetPosition)
                ).withTimeout(600),
                new SetClawAngle(ClawConfiguration.VerticalRotation.DOWN),
                new WaitCommand(150),
                new SetClawTwist(sample.isVerticallyOriented ? ClawConfiguration.HorizontalRotation.NORMAL : ClawConfiguration.HorizontalRotation.FLIPPED),
                new WaitCommand(250),
                new SetClawState(ClawConfiguration.GripperState.CLOSED),
                new WaitCommand(150)
        );
    }
}
