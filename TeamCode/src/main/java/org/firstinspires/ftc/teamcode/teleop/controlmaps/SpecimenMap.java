package org.firstinspires.ftc.teamcode.teleop.controlmaps;

import static org.firstinspires.ftc.teamcode.auto.sample.PointsSample.SUB_GRAB;
import static org.firstinspires.ftc.teamcode.auto.specimen.PointsSpecimen.SUB_GRAB_SPEC;
import static org.firstinspires.ftc.teamcode.auto.specimen.PointsSpecimen.SUB_GRAB_SPEC_CONTROL;
import static org.firstinspires.ftc.teamcode.auto.specimen.PointsSpecimen.SUB_GRAB_SPEC_DEPOSIT;
import static org.firstinspires.ftc.teamcode.helpers.pedro.PoseToPath.bezierPath;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.commands.FollowPath;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;

import org.firstinspires.ftc.teamcode.auto.sample.SubmersibleGrab;
import org.firstinspires.ftc.teamcode.helpers.commands.CustomConditionalCommand;
import org.firstinspires.ftc.teamcode.helpers.commands.InstantCommand;
import org.firstinspires.ftc.teamcode.helpers.commands.LogCommand;
import org.firstinspires.ftc.teamcode.helpers.controls.DriverControls;
import org.firstinspires.ftc.teamcode.helpers.controls.button.ButtonCtl;
import org.firstinspires.ftc.teamcode.helpers.controls.rumble.RumbleControls;
import org.firstinspires.ftc.teamcode.helpers.enums.Alliance;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
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
    }

    private void subGrabAndCycle() {
        // Run sub grab, drop, go back. Transition to grab state after dropping by using LB
        globalMap.followerActive = true;
        f.holdPoint(new Pose(f.getPose().getX(), f.getPose().getY(), SUB_GRAB_SPEC.getHeading()));
        double headingError = Math.abs(f.getPose().getHeading() - SUB_GRAB_SPEC.getHeading());

        // TODO subgrab will not grab in this orientation -- have to switch to using v2
        cs.schedule(
                new SequentialCommandGroup(
                        new SequentialCommandGroup(
                                new LogCommand("SubGrabTeleop", "Heading error: " + headingError),
                                new WaitCommand((long) (headingError * 30)),
                                new SubmersibleGrab(f, Alliance.BLUE, globalMap.reader, rc, true),
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
                                        new FollowPath(f, bezierPath(f.getPose(), SUB_GRAB_SPEC_CONTROL, SUB_GRAB_SPEC_DEPOSIT)
                                                .setLinearHeadingInterpolation(f.getPose().getHeading(), SUB_GRAB_SPEC_DEPOSIT.getHeading())
                                                .build()
                                        ).setCompletionThreshold(0.1),
                                        new CustomConditionalCommand(
                                                new SequentialCommandGroup(
                                                        new WaitUntilCommand(() -> VLRSubsystem.getArm().currentAngleDegrees() > 140),
                                                        new SetClawState(ClawConfiguration.GripperState.OPEN),
                                                        new WaitCommand(200),
                                                        // TODO check if modifier is pressed and transition into hanging
                                                        new SetClawState(ClawConfiguration.GripperState.CLOSED),

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
                                                () -> globalMap.followerActive // do not continue if follower is off, since that probably means that the grab has failed
                                        ),
                                        new SetArmPosition().angleDegrees(0)
                                )
                        )
                )
        );
    }

    private void hangCycle() {
        // Close claw, hang, come back. End loop after hanging by using RT.
    }
}
