package org.firstinspires.ftc.teamcode.commands.specimen;

import static org.firstinspires.ftc.teamcode.Points_specimen.*;
import static org.firstinspires.ftc.teamcode.helpers.pedro.PoseToPath.bezierPath;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.pedropathing.commands.FollowPath;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.SetRotatorAngle;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.SetSlideExtension;
import org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawAngle;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawState;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawTwist;

public class AutonomousPeriodActions extends SequentialCommandGroup {
    public AutonomousPeriodActions(Follower f) {
        addCommands(
                new SetClawAngle(ClawConfiguration.VerticalRotation.UP),
                new SetClawTwist(ClawConfiguration.HorizontalRotation.NORMAL),
                new SetClawState(ClawConfiguration.GripperState.CLOSED),

                //DRIVE TO BAR AND EXTEND ARM
                new ParallelCommandGroup(
                        new FollowPath(f, bezierPath(START_POSE, SCORE_PRELOAD)
                                .setConstantHeadingInterpolation(Math.toRadians(-180)).build()),

                        new SequentialCommandGroup(
                                new SetRotatorAngle(100),
                                new WaitCommand(300),
                                new SetSlideExtension(0.15)
                        )
                ),

                //SCORE PRELOAD SAMPLE------
                new WaitCommand(80),
                new SetSlideExtension(0.43),
                new WaitUntilCommand(()-> VLRSubsystem.getSlides().reachedTargetPosition()),
                new SetClawState(ClawConfiguration.GripperState.OPEN),
                new WaitCommand(100),

                //DRIVE TO FIRST SAMPLE-----
                new ParallelCommandGroup(
                        new FollowPath(f, bezierPath(SCORE_PRELOAD, CONTROL_1, CONTROL_2, PICK_UP_SAMPLE_1)
                                .setTangentHeadingInterpolation().build()),
                        new SequentialCommandGroup(
                                new SetClawState(ClawConfiguration.GripperState.OPEN),
                                new SetSlideExtension(ArmSlideConfiguration.TargetPosition.RETRACTED),
                                new WaitUntilCommand(()-> VLRSubsystem.getSlides().getExtension() < 0.1),
                                new SetRotatorAngle(ArmRotatorConfiguration.TargetAngle.RETRACT),
                                new WaitUntilCommand(()-> VLRSubsystem.getRotator().reachedTargetPosition()),
                                new WaitCommand(600),
                                new SetSlideExtension(0.4),
                                new WaitUntilCommand(()-> VLRSubsystem.getSlides().getExtension() > 0.2),
                                new SetClawAngle(ClawConfiguration.VerticalRotation.DOWN),
                                new WaitUntilCommand(()-> VLRSubsystem.getSlides().reachedTargetPosition())
                        )
                ),

                new WaitCommand(300),
                depositSample(),

                //DRIVE TO SECOND SAMPLE--------
                new ParallelCommandGroup(
                    new FollowPath(f, bezierPath(PICK_UP_SAMPLE_1, PICK_UP_SAMPLE_2)
                            .setConstantHeadingInterpolation(PICK_UP_SAMPLE_2.getHeading()).build()),
                    prepareArmForSpikeMarkSamplePickup()
                ),

                depositSample(),

                //DRIVE TO THIRD SAMPLE-------
                new ParallelCommandGroup(
                        new FollowPath(f, bezierPath(PICK_UP_SAMPLE_2, PICK_UP_SAMPLE_3)
                                .setLinearHeadingInterpolation(PICK_UP_SAMPLE_2.getHeading(), PICK_UP_SAMPLE_3.getHeading()).build()),
                        prepareArmForSpikeMarkSamplePickup()
                ),

                depositSample(),

                //DRIVE TO PICK UP FIRST SPECIMEN-----
                new ParallelCommandGroup(
                        new FollowPath(f, bezierPath(PICK_UP_SAMPLE_3, PICK_UP_SPECIMEN)
                                .setLinearHeadingInterpolation(PICK_UP_SAMPLE_3.getHeading(), PICK_UP_SPECIMEN.getHeading()).build()),
                        new SequentialCommandGroup(
                                prepareArmForSpikeMarkSamplePickup(),
                                new SetClawAngle(0.85)
                        )
                ),

                grabSpecimen(),
                cycle(f, 2),

                grabSpecimen(),
                cycle(f, 3),

                grabSpecimen(),
                cycle(f, 4),

                grabSpecimen(),
                cycle(f, 5)
        );
    }


    SequentialCommandGroup prepareArmForSpikeMarkSamplePickup(){
        return new SequentialCommandGroup(
                new SetRotatorAngle(ArmRotatorConfiguration.TargetAngle.RETRACT),
                new WaitUntilCommand(()-> VLRSubsystem.getRotator().getAngleDegrees() < 20),
                new SetSlideExtension(0.4),
                new WaitUntilCommand(()-> VLRSubsystem.getSlides().getExtension() > 0.2),
                new SetClawAngle(ClawConfiguration.VerticalRotation.DOWN),
                new WaitUntilCommand(()-> VLRSubsystem.getSlides().reachedTargetPosition())
        );
    }



    SequentialCommandGroup depositSample(){
        return new SequentialCommandGroup(
                new WaitCommand(150),
                new SetClawState(ClawConfiguration.GripperState.CLOSED),
                new WaitCommand(130),
                new SetClawAngle(ClawConfiguration.VerticalRotation.UP),
                new SetSlideExtension(ArmSlideConfiguration.TargetPosition.RETRACTED),
                new SetRotatorAngle(180),
                new WaitUntilCommand(()-> VLRSubsystem.getRotator().getAngleDegrees() > 170),
                new SetClawState(ClawConfiguration.GripperState.OPEN)
        );
    }


    SequentialCommandGroup grabSpecimen(){
        return new SequentialCommandGroup(
                new WaitCommand(450),
                new SetSlideExtension(0.6),
                new WaitUntilCommand(()-> VLRSubsystem.getSlides().reachedTargetPosition()),
                new SetClawState(ClawConfiguration.GripperState.CLOSED),
                new WaitCommand(100)
        );
    }


    SequentialCommandGroup cycle(Follower follower, int specimen){
        if (specimen <= 1) throw new IllegalArgumentException("FIRST SPECIMEN IS PRELOAD, CYCLE STARTS WITH SECOND (2)");
        else if (specimen >= 7) throw new IllegalArgumentException("ARE YOU CRAZY?");

        Pose targetSpecimen = SCORE_SPECIMEN;
        targetSpecimen.add(new Pose(0, 1.25d * (specimen - 2)));

        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new FollowPath(follower, bezierPath(PICK_UP_SPECIMEN, targetSpecimen)
                                .setLinearHeadingInterpolation(PICK_UP_SPECIMEN.getHeading(), targetSpecimen.getHeading()).build()),

                        new SequentialCommandGroup(
                                new SetClawAngle(ClawConfiguration.VerticalRotation.UP),
                                new SetSlideExtension(ArmSlideConfiguration.TargetPosition.RETRACTED),
                                new WaitUntilCommand(()-> VLRSubsystem.getSlides().getExtension() < 0.15),

                                new SetRotatorAngle(100),
                                new WaitCommand(500),
                                new SetSlideExtension(0.15)
                        )
                ),

                //SCORE SPECIMEN----
                new WaitCommand(450),
                new SetSlideExtension(0.43),
                new WaitUntilCommand(()-> VLRSubsystem.getSlides().reachedTargetPosition()),
                new SetClawState(ClawConfiguration.GripperState.OPEN),

                //DRIVE BACK
                new ConditionalCommand(
                        driveToHumanPlayerZone(follower, targetSpecimen),

                        //RETRACT ARM AND DON'T DRIVE BACK IF LAST SPECIMEN
                        new SequentialCommandGroup(
                                new SetRotatorAngle(95),
                                new WaitCommand(300),

                                new SetSlideExtension(ArmSlideConfiguration.TargetPosition.RETRACTED),
                                new WaitUntilCommand(()-> VLRSubsystem.getSlides().getExtension() < 0.2),

                                new SetRotatorAngle(ArmRotatorConfiguration.TargetAngle.RETRACT),
                                new WaitUntilCommand(()-> VLRSubsystem.getRotator().reachedTargetPosition())
                        ),
                        ()-> specimen < 5
                )
        );
    }



    ParallelCommandGroup driveToHumanPlayerZone(Follower follower, Pose targetSpecimen){
        return new ParallelCommandGroup(
                new FollowPath(follower, bezierPath(targetSpecimen, PICK_UP_SPECIMEN)
                        .setLinearHeadingInterpolation(targetSpecimen.getHeading(), PICK_UP_SPECIMEN.getHeading()).build()),
                new SequentialCommandGroup(
                        new SetSlideExtension(ArmSlideConfiguration.TargetPosition.RETRACTED),
                        new WaitUntilCommand(()-> VLRSubsystem.getSlides().getExtension() < 0.2),

                        new SetRotatorAngle(ArmRotatorConfiguration.TargetAngle.RETRACT),
                        new WaitUntilCommand(()-> VLRSubsystem.getRotator().reachedTargetPosition()),

                        new SetSlideExtension(0.4),
                        new WaitUntilCommand(()-> VLRSubsystem.getSlides().getExtension() > 0.2),
                        new SetClawAngle(0.85),
                        new WaitUntilCommand(()-> VLRSubsystem.getSlides().reachedTargetPosition())
                )
        );
    }
}