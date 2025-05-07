package org.firstinspires.ftc.teamcode.teleop.controlmaps;

import static org.firstinspires.ftc.teamcode.auto.specimen.PointsSpecimen.DEPOSIT_SAMPLE_3_END;
import static org.firstinspires.ftc.teamcode.auto.specimen.PointsSpecimen.DRIVE_BACK;
import static org.firstinspires.ftc.teamcode.auto.specimen.PointsSpecimen.PICK_UP_SPECIMENS_FROM_HUMAN_PLAYER;
import static org.firstinspires.ftc.teamcode.auto.specimen.PointsSpecimen.SCORE_SECOND_SPECIMEN;
import static org.firstinspires.ftc.teamcode.auto.specimen.PointsSpecimen.SUB_GRAB_SPEC;
import static org.firstinspires.ftc.teamcode.auto.specimen.PointsSpecimen.SUB_GRAB_SPEC_CONTROL;
import static org.firstinspires.ftc.teamcode.auto.specimen.PointsSpecimen.SUB_GRAB_SPEC_DEPOSIT;
import static org.firstinspires.ftc.teamcode.auto.specimen.PointsSpecimen.SUB_GRAB_SPEC_DEPOSIT_TRANSITION;
import static org.firstinspires.ftc.teamcode.auto.specimen.PointsSpecimen.SUB_GRAB_SPEC_DEPOSIT_TRANSITION_CONTROL;
import static org.firstinspires.ftc.teamcode.auto.specimen.PointsSpecimen.TELEOP_SPEC_HANG_TRANSITION;
import static org.firstinspires.ftc.teamcode.auto.specimen.PointsSpecimen.TELEOP_SPEC_HANG_TRANSITION_FINAL;
import static org.firstinspires.ftc.teamcode.helpers.pedro.PoseToPath.bezierPath;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.commands.FollowPath;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;

import org.firstinspires.ftc.teamcode.auto.sample.SubmersibleGrabV2;
import org.firstinspires.ftc.teamcode.helpers.commands.CustomConditionalCommand;
import org.firstinspires.ftc.teamcode.helpers.commands.InstantCommand;
import org.firstinspires.ftc.teamcode.helpers.commands.LogCommand;
import org.firstinspires.ftc.teamcode.helpers.controls.DriverControls;
import org.firstinspires.ftc.teamcode.helpers.controls.button.ButtonCtl;
import org.firstinspires.ftc.teamcode.helpers.controls.rumble.RumbleControls;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmState;
import org.firstinspires.ftc.teamcode.subsystems.arm.SetArmPosition;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawAngle;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawState;

public class SpecimenMap extends ControlMap {
    GlobalMap globalMap;
    CommandScheduler cs;
    Follower f;
    RumbleControls rc;

    public SpecimenMap(DriverControls driverControls, CommandScheduler cs, GlobalMap globalMap) {
        super(driverControls);
        this.cs = cs;
        this.f = globalMap.f;
        this.rc = globalMap.rc;
        this.globalMap = globalMap;
    }

    @Override
    public void initialize() {
        // Basic idea
        // LB starts the loop - sub grab, drop, go back. Have the ability to cancel mid way
        // after dropping lr grabbing using LT - after dropping, gets ready for grab
        // RB is for hanging - close claw, go hang, come back, get rdy for grab, disable follower
        // RT is for cancelling after hang - retract arm and stop the loop.
        gp.add(new ButtonCtl(GamepadKeys.Button.LEFT_BUMPER, ButtonCtl.Trigger.WAS_JUST_PRESSED, (Boolean a) -> {
            subGrabAndCycle();
        }));

        gp.add(new ButtonCtl(GamepadKeys.Button.Y, ButtonCtl.Trigger.WAS_JUST_PRESSED, (Boolean a) -> {
            retract();
        }));

        gp.add(new ButtonCtl(GamepadKeys.Button.RIGHT_STICK_BUTTON, ButtonCtl.Trigger.WAS_JUST_PRESSED, (Boolean a) -> {
            relocalize();
        }));

    }

    private void subGrabAndCycle() {
        // Run sub grab, drop, go back. Transition to grab state after dropping by using LT
        globalMap.followerActive = true;
        f.holdPoint(f.getPose());

        cs.schedule(
                new SequentialCommandGroup(
                        new SequentialCommandGroup(
                                new SubmersibleGrabV2(f, globalMap.reader, rc),
                                new WaitCommand(230),
                                new SetClawState(ClawConfiguration.GripperState.CLOSED),
                                new WaitCommand(150)
                        ),
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new SetArmPosition().retract(),
                                        new SetClawAngle(ClawConfiguration.VerticalRotation.UP),
                                        new SetArmPosition().angleDegrees(155)
                                ),
                                new SequentialCommandGroup(
                                        new WaitCommand(200),
                                        new CustomConditionalCommand(
                                                new CustomConditionalCommand(
                                                        new SequentialCommandGroup(
                                                                new FollowPath(f, bezierPath(f.getPose(), SUB_GRAB_SPEC_DEPOSIT_TRANSITION_CONTROL, SUB_GRAB_SPEC_DEPOSIT_TRANSITION)
                                                                        .setLinearHeadingInterpolation(f.getPose().getHeading(), SUB_GRAB_SPEC_DEPOSIT_TRANSITION.getHeading())
                                                                        .build()),
                                                                new WaitUntilCommand(() -> VLRSubsystem.getArm().currentAngleDegrees() > 140),
                                                                new SetClawState(ClawConfiguration.GripperState.OPEN),
                                                                new WaitCommand(200),

                                                                new SetClawAngle(0.52),
                                                                new WaitCommand(500),
                                                                new InstantCommand() {
                                                                    @Override
                                                                    public void run() {
                                                                        globalMap.followerActive = false;
                                                                        rc.singleBlip();
                                                                    }
                                                                },
                                                                new WaitUntilCommand(() -> gp.gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.3),
                                                                new InstantCommand() {
                                                                    @Override
                                                                    public void run() {
                                                                        globalMap.followerActive = true;
                                                                    }
                                                                },
                                                                new SetClawState(ClawConfiguration.GripperState.CLOSED),
                                                                new WaitCommand(200),
                                                                new ParallelCommandGroup(
                                                                        new WaitCommand(50).andThen(
                                                                                new FollowPath(f, bezierPath(SUB_GRAB_SPEC_DEPOSIT_TRANSITION, TELEOP_SPEC_HANG_TRANSITION)
                                                                                        .setConstantHeadingInterpolation(TELEOP_SPEC_HANG_TRANSITION.getHeading()).build()).setCompletionThreshold(0.9),
                                                                                new FollowPath(f, bezierPath(TELEOP_SPEC_HANG_TRANSITION, TELEOP_SPEC_HANG_TRANSITION_FINAL)
                                                                                        .setConstantHeadingInterpolation(TELEOP_SPEC_HANG_TRANSITION_FINAL.getHeading()).build())
                                                                        ),
                                                                        new ParallelCommandGroup(
                                                                                new SetArmPosition().angleDegrees(100).andThen(new SetArmPosition().extensionAndAngleDegrees(0.53, 50)),
                                                                                new WaitCommand(200).andThen(new SetClawAngle(ClawConfiguration.VerticalRotation.DOWN))
                                                                        ),
                                                                        new WaitUntilCommand(() -> f.getPose().getX() > 33.5).andThen(new SetArmPosition().extensionRelative(0.23)).andThen(new SetArmPosition().setArmState(ArmState.State.SPECIMEN_SCORE_FRONT))
                                                                ),
                                                                new WaitCommand(150),
                                                                new SetClawState(ClawConfiguration.GripperState.OPEN),
                                                                new WaitCommand(150),
                                                                new ParallelCommandGroup(
                                                                        new SetArmPosition().retract(),
                                                                        new FollowPath(f, bezierPath(TELEOP_SPEC_HANG_TRANSITION_FINAL, PICK_UP_SPECIMENS_FROM_HUMAN_PLAYER).setLinearHeadingInterpolation(TELEOP_SPEC_HANG_TRANSITION_FINAL.getHeading(), PICK_UP_SPECIMENS_FROM_HUMAN_PLAYER.getHeading()).build())
                                                                )
                                                        ),
                                                        // NORMAL
                                                        new SequentialCommandGroup(
                                                                new FollowPath(f, bezierPath(f.getPose(), SUB_GRAB_SPEC_CONTROL, SUB_GRAB_SPEC_DEPOSIT)
                                                                        .setLinearHeadingInterpolation(f.getPose().getHeading(), SUB_GRAB_SPEC_DEPOSIT.getHeading())
                                                                        .build()).setCompletionThreshold(0.1),
                                                                new WaitUntilCommand(() -> VLRSubsystem.getArm().currentAngleDegrees() > 140),
                                                                new SetClawState(ClawConfiguration.GripperState.OPEN),
                                                                new WaitCommand(200),

                                                                new SetClawState(ClawConfiguration.GripperState.CLOSED),
                                                                new SetArmPosition().angleDegrees(0),
                                                                new FollowPath(f, bezierPath(SUB_GRAB_SPEC_DEPOSIT, SUB_GRAB_SPEC_CONTROL, SUB_GRAB_SPEC)
                                                                        .setLinearHeadingInterpolation(SUB_GRAB_SPEC_DEPOSIT.getHeading(), SUB_GRAB_SPEC.getHeading())
                                                                        .build()
                                                                ).setCompletionThreshold(0.6),

                                                                new InstantCommand() {
                                                                    @Override
                                                                    public void run() {
                                                                        globalMap.followerActive = false;
                                                                        rc.singleBlip();
                                                                    }
                                                                }
                                                        ),
                                                        () -> gp.gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.3
                                                ),
                                                new SequentialCommandGroup(
                                                        new WaitCommand(250),
                                                        new SetArmPosition().angleDegrees(0)
                                                ),
                                                () -> globalMap.followerActive // do not continue if follower is off, since that probably means that the grab has failed
                                        )
                                )
                        )
                )
        );
    }

    private void hangCycle() {
        // Close claw, hang, come back. End loop after hanging by using RT.
    }

    private void relocalize() {
        // todo relocalize
        rc.rumbleBlips(1);
    }

    private void retract() {
        cs.schedule(new SetArmPosition().retract());
    }
}
