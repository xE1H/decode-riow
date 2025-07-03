package org.firstinspires.ftc.teamcode.auto.specimen;

import static org.firstinspires.ftc.teamcode.auto.specimen.PointsSpecimen.DELTA;
import static org.firstinspires.ftc.teamcode.auto.specimen.PointsSpecimen.DEPOSIT_SAMPLE_3_END;
import static org.firstinspires.ftc.teamcode.auto.specimen.PointsSpecimen.DEPOSIT_SAMPLE_3_START;
import static org.firstinspires.ftc.teamcode.auto.specimen.PointsSpecimen.DRIVE_BACK;
import static org.firstinspires.ftc.teamcode.auto.specimen.PointsSpecimen.MIDPOINT_BEFORE_PICKUP;
import static org.firstinspires.ftc.teamcode.auto.specimen.PointsSpecimen.PICK_UP_SAMPLE_1;
import static org.firstinspires.ftc.teamcode.auto.specimen.PointsSpecimen.PICK_UP_SAMPLE_2;
import static org.firstinspires.ftc.teamcode.auto.specimen.PointsSpecimen.PICK_UP_SAMPLE_3;
import static org.firstinspires.ftc.teamcode.auto.specimen.PointsSpecimen.PICK_UP_SPECIMENS_FROM_HUMAN_PLAYER;
import static org.firstinspires.ftc.teamcode.auto.specimen.PointsSpecimen.SCORE_PRELOAD_AND_SUB_PICKUP;
import static org.firstinspires.ftc.teamcode.auto.specimen.PointsSpecimen.SCORE_SECOND_SPECIMEN;
import static org.firstinspires.ftc.teamcode.auto.specimen.PointsSpecimen.SCORE_SPECIMEN_BACK;
import static org.firstinspires.ftc.teamcode.auto.specimen.PointsSpecimen.SECOND_SPECIMEN_CONTROL;
import static org.firstinspires.ftc.teamcode.auto.specimen.PointsSpecimen.START_POSE;
import static org.firstinspires.ftc.teamcode.helpers.pedro.PoseToPath.bezierPath;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.pedropathing.commands.FollowPath;
import com.pedropathing.commands.HoldPoint;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import org.firstinspires.ftc.teamcode.auto.sample.SubmersibleGrabV2;
import org.firstinspires.ftc.teamcode.helpers.commands.CustomConditionalCommand;
import org.firstinspires.ftc.teamcode.helpers.commands.LogCommand;
import org.firstinspires.ftc.teamcode.helpers.commands.ScheduleRuntimeCommand;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.helpers.utils.GlobalTimer;
import org.firstinspires.ftc.teamcode.pedro.FollowPathRelative;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmState;
import org.firstinspires.ftc.teamcode.subsystems.arm.MainArmConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.arm.ResetRotator;
import org.firstinspires.ftc.teamcode.subsystems.arm.ResetSlides;
import org.firstinspires.ftc.teamcode.subsystems.arm.SetArmPosition;
import org.firstinspires.ftc.teamcode.subsystems.blinkin.SetPattern;
import org.firstinspires.ftc.teamcode.subsystems.chassis.Chassis;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawAngle;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawState;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawTwist;
import org.firstinspires.ftc.teamcode.subsystems.limelight.LimelightYoloReader;
import org.firstinspires.ftc.teamcode.subsystems.limelight.commands.RequestLimelightFrame;
import org.firstinspires.ftc.teamcode.subsystems.limelight.commands.WaitUntilNextLimelightFrame;

import java.util.logging.Level;

public class AutonomousPeriodActionSpecimen extends SequentialCommandGroup {
    private volatile boolean readyForSubPickup = false;
    private final double pathTValueConstraint = 0.92;
    LimelightYoloReader reader;
    private boolean goingFor6th = false;


    public AutonomousPeriodActionSpecimen(Follower follower, LimelightYoloReader reader) {
        this.reader = reader;

        addCommands(
                new SetArmPosition().setArmState(ArmState.State.IN_ROBOT),
                new SetClawTwist(0.3),
                new SetClawAngle(0.065),
                new SetClawState(ClawConfiguration.GripperState.CLOSED),
                //new SetPattern().red(),

                //DRIVE TO BAR AND EXTEND ARM
                new ParallelCommandGroup(
                        new SetArmPosition().scoreSpecimenFront(),
                        new WaitCommand(50).andThen(new FollowPath(follower, bezierPath(START_POSE, SCORE_PRELOAD_AND_SUB_PICKUP)
                                        .setConstantHeadingInterpolation(SCORE_PRELOAD_AND_SUB_PICKUP.getHeading()).setZeroPowerAccelerationMultiplier(5).setPathEndTValueConstraint(0.95).build())),
                        new SequentialCommandGroup(
                                new WaitUntilCommand(()-> follower.atPose(SCORE_PRELOAD_AND_SUB_PICKUP, 1.5, 1.5, Math.toRadians(3))),
                                new SetArmPosition().extensionRelative(0.21),
                                new SetClawState(ClawConfiguration.GripperState.OPEN)

                        )
                ),

                new WaitCommand(30),
                new RequestLimelightFrame(reader, follower),
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new WaitCommand(30),

                                new SetArmPosition().retract().interruptOn(()-> VLRSubsystem.getArm().currentAngleDegrees() < 20),
                                new SetArmPosition().setArmState(ArmState.State.IN_ROBOT),

                                new InstantCommand(()-> readyForSubPickup = true),
                                new SetPattern().green()
                        ),

                        new SequentialCommandGroup(
                                new WaitUntilNextLimelightFrame(reader),
                                new WaitUntilCommand(()-> readyForSubPickup),
                                new SubmersibleGrabV2(follower, reader, true).setNegativeYLimit(64.75)
                        )
                ),

                //SCORING SPIKE MARK SAMPLE INTO HUMAN PLAYER AREA
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new SetArmPosition().retract(),
                                new SetPattern().oceanPalette(),
                                new InstantCommand(()-> VLRSubsystem.getArm().setOperationMode(MainArmConfiguration.OPERATION_MODE.NORMAL_SLOWER)),
                                new ParallelCommandGroup(
                                        new SetArmPosition().angleDegrees(140),
                                        new WaitCommand(400).andThen(new SetClawAngle(0.5))
                                ),
                                new WaitUntilCommand(()-> follower.getPose().getY() < PICK_UP_SAMPLE_1.getY() + 9),
                                new SetClawState(ClawConfiguration.GripperState.OPEN),
                                //new SetPattern().green(),
                                new InstantCommand(()-> VLRSubsystem.getArm().setOperationMode(MainArmConfiguration.OPERATION_MODE.NORMAL)),

                                new SetClawAngle(ClawConfiguration.VerticalRotation.UP),
                                new SetClawTwist(ClawConfiguration.HorizontalRotation.NORMAL),

                                new ParallelCommandGroup(
                                        new SetArmPosition().angleDegrees(2).andThen(new SetPattern().red()),
                                        new SequentialCommandGroup(
                                                new WaitUntilCommand(()-> VLRSubsystem.getArm().currentAngleDegrees() < 32),
                                                new SetArmPosition().extension(0.345)
                                        ),
                                        new SequentialCommandGroup(
                                                new WaitUntilCommand(()-> VLRSubsystem.getArm().currentExtension() > 0.04),
                                                new SetClawAngle(ClawConfiguration.VerticalRotation.DOWN),
                                                new SetArmPosition().angleDegrees(0)
                                        )
                                )
                        ),


                        new SequentialCommandGroup(
                                new WaitCommand(100),
                                new LogCommand("AUTO BOMBO", "STARTING TO MOVE OUT"),

                                new FollowPathRelative(follower, new Pose(-5, 0, 0)),
//                                new org.firstinspires.ftc.teamcode.helpers.commands.InstantCommand() {
//                                    @Override
//                                    public void run() {
//                                        follower.breakFollowing();
//                                    }
//                                },

//                                new ScheduleRuntimeCommand(
//                                        ()-> new FollowPath(follower, bezierPath(follower.getPose(), new Pose(SCORE_PRELOAD_AND_SUB_PICKUP.getX() - 6, follower.getPose().getY(), SCORE_PRELOAD_AND_SUB_PICKUP.getHeading()))
//                                        .setConstantHeadingInterpolation(PICK_UP_SAMPLE_1.getHeading()).setPathEndTValueConstraint(pathTValueConstraint).build())),
                                new LogCommand("AUTO BOMBO", "FIRST PATH PASSED"),
                                new ScheduleRuntimeCommand(
                                        ()-> new FollowPath(follower, bezierPath(follower.getPose(), PICK_UP_SAMPLE_1)
                                        .setConstantHeadingInterpolation(PICK_UP_SAMPLE_1.getHeading()).setZeroPowerAccelerationMultiplier(3).build()))
                        )
                ),

                //FIRST SPIKE MARK SAMPLE
                //new SetPattern().green(),
                new ParallelCommandGroup(
                        //new WaitCommand(300).andThen(new SetPattern().red()),
                        scoreSampleIntoHumanPlayerArea(0.345, 2),
                        new WaitCommand(350).andThen(new FollowPath(follower, bezierPath(PICK_UP_SAMPLE_1, PICK_UP_SAMPLE_2)
                                        .setConstantHeadingInterpolation(PICK_UP_SAMPLE_2.getHeading()).setPathEndTValueConstraint(pathTValueConstraint).build()))
                ),


                //SECOND SPIKE MARK SAMPLE
                //new SetPattern().green(),
                new ParallelCommandGroup(
                        scoreSampleIntoHumanPlayerArea(0.4025, 3),
                        //new WaitCommand(500).andThen(new SetPattern().red()),
                        new WaitCommand(400).andThen(
                                new FollowPath(follower, bezierPath(PICK_UP_SAMPLE_2, PICK_UP_SAMPLE_3)
                                        .setLinearHeadingInterpolation(PICK_UP_SAMPLE_2.getHeading(), PICK_UP_SAMPLE_3.getHeading()).setPathEndTValueConstraint(pathTValueConstraint).build()))
                ),

                //THIRD SPIKE MARK SAMPLE
                //new SetPattern().green(),
                new ParallelCommandGroup(
                        //new WaitCommand(500).andThen(new SetPattern().red()),
                        scoreSampleIntoHumanPlayerArea(0, 0.35, 4),
                        new SequentialCommandGroup(
                                new FollowPath(follower, bezierPath(PICK_UP_SAMPLE_3, DEPOSIT_SAMPLE_3_START)
                                        .setLinearHeadingInterpolation(PICK_UP_SAMPLE_3.getHeading(), DEPOSIT_SAMPLE_3_START.getHeading()).setZeroPowerAccelerationMultiplier(3.5).setPathEndTValueConstraint(pathTValueConstraint).build(), false),

                                new WaitCommand(150),
                                new FollowPath(follower, bezierPath(DEPOSIT_SAMPLE_3_START, DEPOSIT_SAMPLE_3_END)
                                        .setConstantHeadingInterpolation(DEPOSIT_SAMPLE_3_END.getHeading()).setPathEndTValueConstraint(0.96).build())
                        )
                ),

                //new SetPattern().green(),
                new SetClawState(ClawConfiguration.GripperState.CLOSED),
                //new WaitCommand(20),

                scoreSecondSpecimen(follower),
                new WaitCommand(80), //buvo 50

                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new WaitCommand(120),
                                new FollowPath(follower, bezierPath(SCORE_SECOND_SPECIMEN, DRIVE_BACK)
                                        .setConstantHeadingInterpolation(DRIVE_BACK.getHeading()).build(), false),
                                new FollowPath(follower, bezierPath(DRIVE_BACK, MIDPOINT_BEFORE_PICKUP)
                                        .setLinearHeadingInterpolation(DRIVE_BACK.getHeading(), MIDPOINT_BEFORE_PICKUP.getHeading()).setPathEndTValueConstraint(pathTValueConstraint).build(), false),
                                new FollowPath(follower, bezierPath(MIDPOINT_BEFORE_PICKUP, PICK_UP_SPECIMENS_FROM_HUMAN_PLAYER)
                                        .setConstantHeadingInterpolation(PICK_UP_SPECIMENS_FROM_HUMAN_PLAYER.getHeading()).build())
                        ),
                        new SequentialCommandGroup(
                                new SetArmPosition().retract(),
                                new WaitCommand(50),
                                new ResetRotator().alongWith(new ResetSlides()),
                                new SetArmPosition().intakeSpecimen(0.445)
                        )
                ),

                cycle(follower, 3),
                cycle(follower, 4),
                cycle(follower, 5),

//                SKIP 6TH IF NO TIME
                new ConditionalCommand(
                        cycle(follower, 6),
                        new LogCommand("skibidi auto logger", "skipped 6th cycle"),
                        ()-> goingFor6th
                )
        );
    }


    private Command cycle(Follower follower, int sample){
        Pose targetStart = new Pose(SCORE_SPECIMEN_BACK.getX(), SCORE_SPECIMEN_BACK.getY() + (sample - 3) * DELTA, SCORE_SPECIMEN_BACK.getHeading());
        Pose targetEnd = new Pose(targetStart.getX() + 6.1 - (sample - 3) * 0.2, targetStart.getY(), targetStart.getHeading());

        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new WaitCommand(120),
                                new FollowPath(follower, bezierPath(PICK_UP_SPECIMENS_FROM_HUMAN_PLAYER, targetStart)
                                        .setLinearHeadingInterpolation(PICK_UP_SPECIMENS_FROM_HUMAN_PLAYER.getHeading(), targetStart.getHeading()).setZeroPowerAccelerationMultiplier(6).setPathEndTValueConstraint(0.94).build()),
                                new LogCommand("SPECIMEN PERIOD ACTIONS", Level.INFO, "SKIBIDI FIRST FOLLOW PATH PASSED"),
//                                new FollowPath(follower, bezierPath(targetStart, targetEnd)
//                                        .setConstantHeadingInterpolation(targetStart.getHeading()).setZeroPowerAccelerationMultiplier(6).build())
//                                        .interruptOn(() -> VLRSubsystem.getInstance(Chassis.class).getBackDistance() < 370),
                                new FollowPath(follower, bezierPath(targetStart, targetEnd)
                                        .setConstantHeadingInterpolation(targetStart.getHeading()).setZeroPowerAccelerationMultiplier(5).setPathEndTValueConstraint(0.9).build())
                                //new LogCommand("BACK DISTANCE LOGGER", Level.INFO, () -> "BACK SENSOR DISTANCE: " + VLRSubsystem.getInstance(Chassis.class).getBackDistance()),
                                //new InstantCommand(follower::breakFollowing),
                        ),
                        new SetArmPosition().scoreSpecimenBack()
                ),

                GlobalTimer.logTime("TIME BEFORE SCORING " + sample + "th SPECIMEN: "),

                new ConditionalCommand(
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new SetArmPosition().extensionRelative(0.22),
                                        new CustomConditionalCommand(
                                                new InstantCommand(()-> goingFor6th = true).andThen(new LogCommand("SKIBIDI", Level.SEVERE, "GOING FOR 6TH")),
                                                ()-> sample <= 5
                                        ),
                                        new SetPattern().blinkSampleColour(RevBlinkinLedDriver.BlinkinPattern.GREEN),
                                        new SetArmPosition().intakeSpecimen(0.485),
                                        GlobalTimer.logTime("TIME BEFORE PICKING UP " + (sample + 1) + "th SPECIMEN: ")
                                ),


                                new SequentialCommandGroup(
                                        new WaitCommand(350),
                                        new InstantCommand(follower::resumePathFollowing),

                                        new ScheduleRuntimeCommand(()-> new FollowPath(follower, bezierPath(follower.getPose(), MIDPOINT_BEFORE_PICKUP)
                                                .setLinearHeadingInterpolation(follower.getPose().getHeading(), MIDPOINT_BEFORE_PICKUP.getHeading()).build(), false)),

                                        new FollowPath(follower, bezierPath(MIDPOINT_BEFORE_PICKUP, PICK_UP_SPECIMENS_FROM_HUMAN_PLAYER)
                                                .setLinearHeadingInterpolation(MIDPOINT_BEFORE_PICKUP.getHeading(), PICK_UP_SPECIMENS_FROM_HUMAN_PLAYER.getHeading()).build()
                                        )
                                        //new SetPattern().blinkSampleColour(RevBlinkinLedDriver.BlinkinPattern.RED)
                                )
                        ),
                        new SequentialCommandGroup(
                                new SetArmPosition().extensionRelative(0.23),//.andThen(new SetPattern().green()),
                                new SetClawState(ClawConfiguration.GripperState.OPEN),
                                new WaitCommand(90),
                                new SetArmPosition().extensionAndAngleDegrees(0, 96)
                        ),
                        ()-> sample <= 5 && GlobalTimer.time() < 25.8
                )
        );
    }



    private Command scoreSecondSpecimen(Follower follower){
        return new ParallelCommandGroup(
                new WaitCommand(50).andThen(
                        new FollowPath(follower, bezierPath(DEPOSIT_SAMPLE_3_END, SECOND_SPECIMEN_CONTROL, SCORE_SECOND_SPECIMEN)
                                .setConstantHeadingInterpolation(DRIVE_BACK.getHeading()).setZeroPowerAccelerationMultiplier(6).build()),
                        new org.firstinspires.ftc.teamcode.helpers.commands.InstantCommand() {
                            @Override
                            public void run() {
                                follower.holdPoint(SCORE_SECOND_SPECIMEN);
                            }
                        }
                ),

                new WaitCommand(200).andThen(new SetClawAngle(ClawConfiguration.VerticalRotation.DOWN)),

                new SequentialCommandGroup(
                        new SetArmPosition().angleDegrees(100),
                        new SetArmPosition().extensionAndAngleDegrees(0.53 , 54.5),
                        new WaitUntilCommand(()-> follower.atPose(SCORE_SECOND_SPECIMEN, 2, 2, Math.toRadians(3.5))),
                        new WaitCommand(40),
                        new SetArmPosition().extensionRelative(0.205),
                        //new SetPattern().green(),
                        new SetArmPosition().setArmState(ArmState.State.SPECIMEN_SCORE_FRONT)
                )
        );
    }

    private Command scoreSampleIntoHumanPlayerArea(double extension, int sample){
        return scoreSampleIntoHumanPlayerArea(extension, ClawConfiguration.HorizontalRotation.NORMAL.pos, sample);
    }


    private Command scoreSampleIntoHumanPlayerArea(double extension, double twist, int sample){
        return new ParallelCommandGroup(
                new SequentialCommandGroup(
                        new SetClawState(ClawConfiguration.GripperState.CLOSED),
                        new WaitCommand(20),
                        new ParallelCommandGroup(
                                new SetArmPosition().extension(0),
                                new SetClawTwist(1),
                                new WaitCommand(50).andThen(new SetClawAngle(ClawConfiguration.VerticalRotation.UP))
                        ),
                        new SetArmPosition().setArmState(ArmState.State.IN_ROBOT)
                ),
                new SequentialCommandGroup(
                        new WaitUntilCommand(()-> VLRSubsystem.getArm().currentExtension() < 0.25),
                        new ParallelCommandGroup(
                                new WaitCommand(100).andThen(new SetClawTwist(ClawConfiguration.HorizontalRotation.NORMAL)),
                                new SequentialCommandGroup(
                                        new SetArmPosition().angleDegrees(168),
                                        new CustomConditionalCommand(
                                                new ParallelCommandGroup(
                                                        new WaitCommand(100).andThen(new SetClawAngle(ClawConfiguration.VerticalRotation.UP)),
                                                        new SetArmPosition().angleDegrees(2),
                                                        new SequentialCommandGroup(
                                                                new WaitUntilCommand(()-> VLRSubsystem.getArm().currentAngleDegrees() < 50),
                                                                new ParallelCommandGroup(
                                                                        new SetArmPosition().extension(extension),
                                                                        new SetClawTwist(twist),
                                                                        new SequentialCommandGroup(
                                                                                new WaitCommand(20),
                                                                                new SetClawAngle(ClawConfiguration.VerticalRotation.DOWN),
                                                                                new WaitCommand(100),
                                                                                new SetArmPosition().angleDegrees(0),
                                                                                new WaitCommand(50)
                                                                        )
                                                                )
                                                        )
                                                ),
                                                ()-> sample != 4
                                        )
                                ),
                                new WaitUntilCommand(()-> VLRSubsystem.getArm().currentAngleDegrees() > 80).andThen(new SetClawAngle(0.475)),
                                new WaitUntilCommand(()-> VLRSubsystem.getArm().currentAngleDegrees() > 130).andThen(new SetClawState(ClawConfiguration.GripperState.OPEN))
                        )
                )
        );
    }
}