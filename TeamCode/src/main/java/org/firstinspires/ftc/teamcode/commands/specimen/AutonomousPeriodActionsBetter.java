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
import org.firstinspires.ftc.teamcode.commands.sample.SubmersibleGrab;
import org.firstinspires.ftc.teamcode.helpers.enums.Alliance;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.SetRotatorAngle;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.SetSlideExtension;
import org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawAngle;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawState;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawTwist;

public class AutonomousPeriodActionsBetter extends SequentialCommandGroup {
    public AutonomousPeriodActionsBetter(Follower f) {
        addCommands(
                new SetClawAngle(ClawConfiguration.VerticalRotation.UP),
                new SetClawTwist(ClawConfiguration.HorizontalRotation.NORMAL),
                new SetClawState(ClawConfiguration.GripperState.CLOSED),

                //DRIVE TO BAR AND EXTEND ARM
                new ParallelCommandGroup(
                        new FollowPath(f, bezierPath(START_POSE, SCORE_PRELOAD_AND_SUB_PICKUP)
                                .setConstantHeadingInterpolation(SCORE_PRELOAD_AND_SUB_PICKUP.getHeading()).build()),

                        new SequentialCommandGroup(
                                new SetRotatorAngle(65),
                                new WaitCommand(300),
                                new SetSlideExtension(0.35)
                        )
                ),

                //SCORE PRELOAD SAMPLE------
                new WaitCommand(80),
                new SetClawState(ClawConfiguration.GripperState.OPEN),
                new WaitCommand(120),

                //RETRACT ARM AND PICK SAMPLE FROM SUB
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new SetSlideExtension(ArmSlideConfiguration.TargetPosition.RETRACTED),
                                new WaitUntilCommand(()-> VLRSubsystem.getSlides().reachedTargetPosition()),
                                new SetRotatorAngle(ArmRotatorConfiguration.TargetAngle.RETRACT),
                                new WaitUntilCommand(()-> VLRSubsystem.getRotator().reachedTargetPosition())
                        ),
                        new SequentialCommandGroup(
                                new WaitCommand(800),
                                new SubmersibleGrab(f, Alliance.BLUE)
                        )
                ),

                //RETRACT ARM FROM SUB AND DRIVE TO LEAVE ONE SUB SAMPLE IN HUMAN PLAYER AREA
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                //RETRACT ARM AFTER SUB GRAB COMMAND
                                new SetClawAngle(ClawConfiguration.VerticalRotation.UP),
                                new SetSlideExtension(ArmSlideConfiguration.TargetPosition.RETRACTED),
                                new WaitCommand(100),
                                new SetClawTwist(ClawConfiguration.HorizontalRotation.NORMAL),
                                new WaitUntilCommand(()-> VLRSubsystem.getSlides().reachedTargetPosition()),

                                //NOW THE ROBOT PREPARES TO DROP SUB SAMPLE INTO HUMAN PLAYER ZONE WHILE DRIVING
                                new WaitCommand(500),
                                new SetRotatorAngle(180),
                                new WaitUntilCommand(()-> VLRSubsystem.getRotator().reachedTargetPosition()),
                                new WaitUntilCommand(()-> f.getPose().getY() < 15),
                                new SetClawState(ClawConfiguration.GripperState.OPEN),
                                new WaitCommand(100),

                                prepareArmForSpikeMarkSamplePickup()
                        ),
                        new SequentialCommandGroup(
                                new FollowPath(f, bezierPath(f.getPose(), POINT_1)
                                        .setConstantHeadingInterpolation(POINT_1.getHeading()).build()),
                                new FollowPath(f, bezierPath(POINT_1, PICK_UP_SAMPLE_1)
                                        .setConstantHeadingInterpolation(PICK_UP_SAMPLE_1.getHeading()).build())
                        )
                ),

                //DRIVE TO SECOND SAMPLE AND DEPOSIT FIRST--------
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new WaitCommand(200),
                                new FollowPath(f, bezierPath(PICK_UP_SAMPLE_1, PICK_UP_SAMPLE_2)
                                        .setConstantHeadingInterpolation(PICK_UP_SAMPLE_2.getHeading()).build())
                        ),
                        new SequentialCommandGroup(
                                depositSample(),
                                prepareArmForSpikeMarkSamplePickup()
                        )
                ),

                //DRIVE TO THIRD SAMPLE WHILE DEPOSITING SECOND-------
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new WaitCommand(200),
                                new FollowPath(f, bezierPath(PICK_UP_SAMPLE_2, PICK_UP_SAMPLE_3)
                                        .setLinearHeadingInterpolation(PICK_UP_SAMPLE_2.getHeading(), PICK_UP_SAMPLE_3.getHeading()).build())
                        ),
                        new SequentialCommandGroup(
                                depositSample(),
                                prepareArmForSpikeMarkSamplePickup()
                        )
                ),

                //DRIVE TO THIRD SAMPLE WHILE DEPOSITING SECOND, TAKE AND SCORE SECOND SPECIMEN-------
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new WaitCommand(200),
                                new FollowPath(f, bezierPath(PICK_UP_SAMPLE_3, DEPOSIT_SAMPLE_3)
                                        .setLinearHeadingInterpolation(PICK_UP_SAMPLE_3.getHeading(), DEPOSIT_SAMPLE_3.getHeading()).build()),
                                new FollowPath(f, bezierPath(DEPOSIT_SAMPLE_3, PICK_UP_SECOND_SPECIMEN_START)
                                        .setConstantHeadingInterpolation(PICK_UP_SECOND_SPECIMEN_START.getHeading()).build()),
                                new FollowPath(f, bezierPath(PICK_UP_SECOND_SPECIMEN_START, PICK_UP_SECOND_SPECIMEN_END)
                                        .setConstantHeadingInterpolation(PICK_UP_SECOND_SPECIMEN_START.getHeading()).build()),
                                new FollowPath(f, bezierPath(PICK_UP_SECOND_SPECIMEN_END, SCORE_SECOND_SPECIMEN)
                                        .setConstantHeadingInterpolation(SCORE_SECOND_SPECIMEN.getHeading()).build())
                        ),
                        new SequentialCommandGroup(
                                depositSample(),
                                new WaitCommand(100),
                                new SetRotatorAngle(150),
                                new SetSlideExtension(ArmSlideConfiguration.TargetPosition.RETRACTED),
                                new SetClawAngle(0.5),
                                new WaitUntilCommand(()-> f.getPose().getY() > 11),
                                new SetClawState(ClawConfiguration.GripperState.CLOSED),
                                new WaitCommand(80),
                                new SetRotatorAngle(65),
                                new WaitUntilCommand(()-> VLRSubsystem.getRotator().reachedTargetPosition()),
                                new SetClawAngle(ClawConfiguration.VerticalRotation.DOWN),
                                new SetSlideExtension(0.35),
                                new WaitUntilCommand(()-> VLRSubsystem.getSlides().reachedTargetPosition())
                        )
                ),

                new WaitCommand(100),
                new SetRotatorAngle(70),
                new WaitCommand(500),

                //RETRACT ARM, PICK ANOTHER SAMPLE FROM SUB
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new SetClawState(ClawConfiguration.GripperState.OPEN),
                                new WaitCommand(150),
                                new SetSlideExtension(ArmSlideConfiguration.TargetPosition.RETRACTED),
                                new WaitUntilCommand(()-> VLRSubsystem.getSlides().reachedTargetPosition()),
                                new SetRotatorAngle(ArmRotatorConfiguration.TargetAngle.RETRACT),
                                new WaitUntilCommand(()-> VLRSubsystem.getRotator().reachedTargetPosition())
                        ),
                        new SequentialCommandGroup(
                                new WaitCommand(900),
                                new SubmersibleGrab(f, Alliance.BLUE)
                        )
                ),

                //RETRACT ARM FROM SUB AND DRIVE TO PICK THIRD SPECIMEN AND DEPOSIT SECOND SAMPLE
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                //RETRACT ARM AFTER SUB GRAB COMMAND
                                new SetClawAngle(ClawConfiguration.VerticalRotation.UP),
                                new SetSlideExtension(ArmSlideConfiguration.TargetPosition.RETRACTED),
                                new WaitCommand(100),
                                new SetClawTwist(ClawConfiguration.HorizontalRotation.NORMAL),
                                new WaitUntilCommand(()-> VLRSubsystem.getSlides().reachedTargetPosition()),

                                //NOT THE ROBOT PREPARES TO DROP SUB SAMPLE INTO HUMAN PLAYER ZONE WHILE DRIVING
                                new WaitCommand(1500),
                                new SetRotatorAngle(20),
                                new SetSlideExtension(0.6),
                                new WaitCommand(50),
                                new SetClawAngle(0.5),
                                new WaitUntilCommand(()-> VLRSubsystem.getRotator().reachedTargetPosition()),
                                new WaitUntilCommand(()-> VLRSubsystem.getSlides().reachedTargetPosition())
                        ),

                        new SequentialCommandGroup(
                                new FollowPath(f, bezierPath(f.getPose(), POINT_1)
                                        .setConstantHeadingInterpolation(POINT_1.getHeading()).build()),
                                new FollowPath(f, bezierPath(POINT_1, PICK_UP_THIRD_SPECIMEN)
                                        .setLinearHeadingInterpolation(POINT_1.getHeading(), PICK_UP_THIRD_SPECIMEN.getHeading()).build())
                        )
                ),

                //DEPOSIT SECOND SUB SAMPLE, PICK UP THIRD SPECIMEN
                new SetClawState(ClawConfiguration.GripperState.OPEN),
                new SetSlideExtension(0.3),
                new WaitCommand(200),
                new SetRotatorAngle(ArmRotatorConfiguration.TargetAngle.RETRACT),
                new SetClawAngle(0.85),
                new WaitUntilCommand(()-> VLRSubsystem.getRotator().reachedTargetPosition()),
                new SetSlideExtension(0.4),
                new WaitUntilCommand(()-> VLRSubsystem.getSlides().reachedTargetPosition()),
                new SetClawState(ClawConfiguration.GripperState.CLOSED),
                new WaitCommand(100),

                //START CYCLING FROM HERE ON
                cycle(f, 3),

                grabSpecimen(),
                cycle(f, 4),

                grabSpecimen(),
                cycle(f, 5),

                grabSpecimen(),
                cycle(f, 6),

                grabSpecimen(),
                cycle(f, 7)
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
        if (specimen <= 1) throw new IllegalArgumentException("FIRST SPECIMEN IS PRELOAD, SECOND USES CUSTOM PATH, CYCLING STARTS WITH THIRD (3)");
        else if (specimen >= 8) throw new IllegalArgumentException("ARE YOU CRAZY?");

        Pose targetSpecimen = SCORE_OTHER_SPECIMENS;
        targetSpecimen.add(new Pose(0, 1.25d * (specimen - 3)));

        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new FollowPath(follower, bezierPath(PICK_UP_OTHER_SPECIMENS, targetSpecimen)
                                .setLinearHeadingInterpolation(PICK_UP_OTHER_SPECIMENS.getHeading(), targetSpecimen.getHeading()).build()),

                        new SequentialCommandGroup(
                                new SetClawAngle(ClawConfiguration.VerticalRotation.UP),
                                new SetSlideExtension(ArmSlideConfiguration.TargetPosition.RETRACTED),
                                new WaitUntilCommand(()-> VLRSubsystem.getSlides().getExtension() < 0.15),

                                new SetRotatorAngle(110),
                                new WaitCommand(500),
                                new SetSlideExtension(0.25)
                        )
                ),

                //SCORE SPECIMEN----
                new WaitCommand(450),
                new SetSlideExtension(0.43),
                new WaitCommand(300),
                new SetClawState(ClawConfiguration.GripperState.OPEN),
                new WaitCommand(120),

                //DRIVE BACK
                new ConditionalCommand(
                        driveToHumanPlayerZone(follower, targetSpecimen),

                        //RETRACT ARM AND DON'T DRIVE BACK IF LAST SPECIMEN
                        new SequentialCommandGroup(
                                new SetRotatorAngle(100),
                                new WaitCommand(300),

                                new SetSlideExtension(ArmSlideConfiguration.TargetPosition.RETRACTED),
                                new WaitUntilCommand(()-> VLRSubsystem.getSlides().getExtension() < 0.2),

                                new SetRotatorAngle(ArmRotatorConfiguration.TargetAngle.RETRACT),
                                new WaitUntilCommand(()-> VLRSubsystem.getRotator().reachedTargetPosition())
                        ),
                        ()-> specimen < 7
                )
        );
    }



    ParallelCommandGroup driveToHumanPlayerZone(Follower follower, Pose targetSpecimen){
        return new ParallelCommandGroup(
                new FollowPath(follower, bezierPath(targetSpecimen, PICK_UP_OTHER_SPECIMENS)
                        .setLinearHeadingInterpolation(targetSpecimen.getHeading(), PICK_UP_OTHER_SPECIMENS.getHeading()).build()),
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