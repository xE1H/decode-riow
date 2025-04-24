package org.firstinspires.ftc.teamcode.commands.specimen;//package org.firstinspires.ftc.teamcode.commands.specimen;

import static org.firstinspires.ftc.teamcode.Points_specimen.*;
import static org.firstinspires.ftc.teamcode.helpers.pedro.PoseToPath.bezierPath;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.pedropathing.commands.FollowPath;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.helpers.commands.CustomConditionalCommand;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.SetArmPosition;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawAngle;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawState;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawTwist;

public class AutonomousPeriodActionSpecimen extends SequentialCommandGroup {
    private boolean firstSpikeMarkSamplePickedUp = false;
    private boolean secondSpikeMarkSamplePickedUp = false;
    private boolean thirdSpikeMarkSamplePickedUP = false;

    public AutonomousPeriodActionSpecimen(Follower follower) {
        addCommands(
                new SetClawAngle(ClawConfiguration.VerticalRotation.UP),
                new SetClawTwist(ClawConfiguration.HorizontalRotation.NORMAL),
                new SetClawState(ClawConfiguration.GripperState.CLOSED),

                //DRIVE TO BAR AND EXTEND ARM
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new SetArmPosition().scoreSpecimenFront(),
                                new SetArmPosition().angleDegrees(50).alongWith(
                                        new WaitUntilCommand(()-> follower.getPose().getX() > 35).andThen(new SetArmPosition().extensionRelative(0.17))
                                )
                        ),

                        new FollowPath(follower, bezierPath(START_POSE, SCORE_PRELOAD_AND_SUB_PICKUP)
                                .setConstantHeadingInterpolation(SCORE_PRELOAD_AND_SUB_PICKUP.getHeading()).build())
                ),
//                new SetArmPosition().extensionRelative(0.2),

                //SCORING SPIKE MARK SAMPLE INTO HUMAN PLAYER AREA
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new SetArmPosition().retract(),
                                new SetArmPosition().intakeSample(0.35),

                                new WaitUntilCommand(()-> follower.getPose().getY() < 17),
                                new InstantCommand(()-> firstSpikeMarkSamplePickedUp = true),

                                scoreSampleIntoHumanPlayerArea().andThen(new SetArmPosition().intakeSample(0.35)),

                                new WaitUntilCommand(()-> follower.getPose().getY() < 7),
                                new InstantCommand(()-> this.secondSpikeMarkSamplePickedUp = true),
                                scoreSampleIntoHumanPlayerArea().andThen(new SetArmPosition().intakeSample(0.43)),


                                new WaitUntilCommand(()-> (follower.getPose().getY() < 6 && follower.headingError < Math.toRadians(8))),
                                new InstantCommand(()-> this.thirdSpikeMarkSamplePickedUP = true),

                                new SetArmPosition().retract(),

                                new ParallelCommandGroup(
                                        new SetArmPosition().angleDegrees(150),
                                        new WaitUntilCommand(()-> VLRSubsystem.getArm().currentAngleDegrees() > 140).andThen(new SetClawState(ClawConfiguration.GripperState.OPEN))
                                ),
                                new SetClawAngle(0.5)
                        ),


                        new SequentialCommandGroup(
                                new WaitCommand(500),
                                new FollowPath(follower, bezierPath(SCORE_PRELOAD_AND_SUB_PICKUP, PICK_UP_SAMPLE_1)
                                        .setConstantHeadingInterpolation(PICK_UP_SAMPLE_1.getHeading()).build()),


                                new WaitUntilCommand(()-> firstSpikeMarkSamplePickedUp),
                                new WaitCommand(200),
                                new FollowPath(follower, bezierPath(PICK_UP_SAMPLE_1, PICK_UP_SAMPLE_2)
                                        .setConstantHeadingInterpolation(PICK_UP_SAMPLE_2.getHeading()).build()),

                                new WaitUntilCommand(()-> this.secondSpikeMarkSamplePickedUp),
                                new WaitCommand(200),
                                new FollowPath(follower, bezierPath(PICK_UP_SAMPLE_2, PICK_UP_SAMPLE_3)
                                        .setLinearHeadingInterpolation(PICK_UP_SAMPLE_2.getHeading(), PICK_UP_SAMPLE_3.getHeading()).build()),

                                new WaitUntilCommand(()-> this.thirdSpikeMarkSamplePickedUP),
                                new WaitCommand(200),
                                new FollowPath(follower, bezierPath(PICK_UP_SAMPLE_3, DEPOSIT_SAMPLE_3)
                                        .setLinearHeadingInterpolation(PICK_UP_SAMPLE_3.getHeading(), DEPOSIT_SAMPLE_3.getHeading()).build())
                        )
                ),
//                new ParallelCommandGroup(
//                        new WaitCommand(400).andThen(
//                            new FollowPath(follower, bezierPath(PICK_UP_SAMPLE_1, PICK_UP_SAMPLE_2)
//                                    .setConstantHeadingInterpolation(PICK_UP_SAMPLE_2.getHeading()).build())
//                        ),
//                        scoreSampleIntoHumanPlayerArea().andThen(new SetArmPosition().intakeSample(0.36))
//                ),
//
//                new WaitCommand(200),
//                new ParallelCommandGroup(
//                        new WaitCommand(400).andThen(
//                                new FollowPath(follower, bezierPath(PICK_UP_SAMPLE_2, PICK_UP_SAMPLE_3)
//                                        .setLinearHeadingInterpolation(PICK_UP_SAMPLE_2.getHeading(), PICK_UP_SAMPLE_3.getHeading()).build())
//                        ),
//                        scoreSampleIntoHumanPlayerArea().andThen(new SetArmPosition().intakeSample(0.43))
//
//                ),
//
//                new WaitCommand(200),
//                new ParallelCommandGroup(
//                        new WaitCommand(400).andThen(
//                                new FollowPath(follower, bezierPath(PICK_UP_SAMPLE_3, DEPOSIT_SAMPLE_3)
//                                        .setLinearHeadingInterpolation(PICK_UP_SAMPLE_3.getHeading(), DEPOSIT_SAMPLE_3.getHeading()).build())
//                        ),
//                        new SequentialCommandGroup(
//                                new SetArmPosition().retract(),
//                                new WaitCommand(100),
//
//                                new ParallelCommandGroup(
//                                        new SetArmPosition().angleDegrees(150),
//                                        new WaitUntilCommand(()-> VLRSubsystem.getArm().currentAngleDegrees() > 100).andThen(new SetClawState(ClawConfiguration.GripperState.OPEN))
//                                ),
//                                new SetClawAngle(0.5)
//                        )
//                ),

                new WaitCommand(1000),
                new SetClawState(ClawConfiguration.GripperState.CLOSED),
                new WaitCommand(100),

                new ParallelCommandGroup(
                        new WaitCommand(100).andThen(
                                new FollowPath(follower, bezierPath(DEPOSIT_SAMPLE_3, SCORE_SECOND_SPECIMEN)
                                        .setConstantHeadingInterpolation(SCORE_SECOND_SPECIMEN.getHeading()).build())
                        ),
                        new SequentialCommandGroup(
                                new SetArmPosition().angleDegrees(0).alongWith(new WaitCommand(200).andThen(new SetClawAngle(ClawConfiguration.VerticalRotation.UP))),
                                new SetArmPosition().scoreSpecimenFront(),
                                new SetArmPosition().angleDegrees(50),
                                new SetArmPosition().extensionRelative(0.17)

                        )
                ),

                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new WaitCommand(110),
                                new FollowPath(follower, bezierPath(SCORE_SECOND_SPECIMEN, DRIVE_BACK)
                                        .setConstantHeadingInterpolation(DRIVE_BACK.getHeading()).build()),
                                new FollowPath(follower, bezierPath(DRIVE_BACK, PICK_UP_SPECIMENS_FROM_HUMAN_PLAYER)
                                        .setLinearHeadingInterpolation(DRIVE_BACK.getHeading(), PICK_UP_SPECIMENS_FROM_HUMAN_PLAYER.getHeading()).build())
                        ),
                        new SequentialCommandGroup(
                                new SetArmPosition().retract(),
                                new WaitCommand(700),
                                new SetArmPosition().intakeSpecimen(0.4)
                        )
                ),


                new ParallelCommandGroup(
                        new WaitCommand(500).andThen(
                                new FollowPath(follower, bezierPath(PICK_UP_SPECIMENS_FROM_HUMAN_PLAYER, SCORE_SPECIMEN_BACK)
                                        .setLinearHeadingInterpolation(PICK_UP_SPECIMENS_FROM_HUMAN_PLAYER.getHeading(), SCORE_SPECIMEN_BACK.getHeading()).build())
                        ),
                        new SequentialCommandGroup(
                                new SetArmPosition().retract(),
                                new SetArmPosition().scoreSpecimenBack()
                        )
                ),

                new SetArmPosition().extensionRelative(0.16),

                new ParallelCommandGroup(
                        new WaitCommand(200).andThen(
                                new FollowPath(follower, bezierPath(SCORE_SPECIMEN_BACK, PICK_UP_SPECIMENS_FROM_HUMAN_PLAYER)
                                        .setLinearHeadingInterpolation(SCORE_SPECIMEN_BACK.getHeading(), PICK_UP_SPECIMENS_FROM_HUMAN_PLAYER.getHeading()).build())
                        ),
                        new SequentialCommandGroup(
                                new SetArmPosition().retract(),
                                new SetArmPosition().intakeSpecimen(0.4)
                        )
                ),

                new WaitCommand(99999999),



                //SCORE FOURTH
                new ParallelCommandGroup(
                        new WaitCommand(300).andThen(
                                new FollowPath(follower, bezierPath(PICK_UP_SPECIMENS_FROM_HUMAN_PLAYER, SCORE_SPECIMEN_BACK_4)
                                        .setLinearHeadingInterpolation(PICK_UP_SPECIMENS_FROM_HUMAN_PLAYER.getHeading(), SCORE_SPECIMEN_BACK_4.getHeading()).build())
                        ),
                        new SequentialCommandGroup(
                                new SetArmPosition().retract(),
                                new SetArmPosition().scoreSpecimenBack()
                        )
                ),

                new WaitCommand(30),
                new SetArmPosition().extensionRelative(0.16),

                new ParallelCommandGroup(
                        new WaitCommand(200).andThen(
                                new FollowPath(follower, bezierPath(SCORE_SPECIMEN_BACK, PICK_UP_SPECIMENS_FROM_HUMAN_PLAYER)
                                        .setLinearHeadingInterpolation(SCORE_SPECIMEN_BACK.getHeading(), PICK_UP_SPECIMENS_FROM_HUMAN_PLAYER.getHeading()).build())
                        ),
                        new SequentialCommandGroup(
                                new SetArmPosition().retract(),
                                new SetArmPosition().intakeSpecimen(0.4)
                        )
                ),


                new ParallelCommandGroup(
                        new WaitCommand(300).andThen(
                                new FollowPath(follower, bezierPath(PICK_UP_SPECIMENS_FROM_HUMAN_PLAYER, SCORE_SPECIMEN_BACK_5)
                                        .setLinearHeadingInterpolation(PICK_UP_SPECIMENS_FROM_HUMAN_PLAYER.getHeading(), SCORE_SPECIMEN_BACK_5.getHeading()).build())
                        ),
                        new SequentialCommandGroup(
                                new SetArmPosition().retract(),
                                new SetArmPosition().scoreSpecimenBack()
                        )
                ),

                new WaitCommand(30),
                new SetArmPosition().extensionRelative(0.16),
                new SetArmPosition().retract()
        );
    }



    private Command scoreSampleIntoHumanPlayerArea(){
        return new SequentialCommandGroup(
                new SetArmPosition().retract(),
                new ParallelCommandGroup(
                        new SetArmPosition().angleDegrees(160),
                        new WaitUntilCommand(()-> VLRSubsystem.getArm().currentAngleDegrees() > 145).andThen(new SetClawState(ClawConfiguration.GripperState.OPEN))
                ),
                new SetArmPosition().angleDegrees(0)
        );
    }
}