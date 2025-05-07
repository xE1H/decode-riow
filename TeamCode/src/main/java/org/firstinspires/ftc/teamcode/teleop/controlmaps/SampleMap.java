package org.firstinspires.ftc.teamcode.teleop.controlmaps;

import static org.firstinspires.ftc.teamcode.auto.sample.PointsSample.BUCKET_HIGH_SCORE_POSE;
import static org.firstinspires.ftc.teamcode.auto.sample.PointsSample.SUB_GRAB;
import static org.firstinspires.ftc.teamcode.auto.sample.PointsSample.SUB_GRAB_0;
import static org.firstinspires.ftc.teamcode.helpers.pedro.PoseToPath.bezierPath;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.commands.FollowPath;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.auto.sample.SubmersibleGrabV2;
import org.firstinspires.ftc.teamcode.helpers.commands.InstantCommand;
import org.firstinspires.ftc.teamcode.helpers.commands.LogCommand;
import org.firstinspires.ftc.teamcode.helpers.controls.DriverControls;
import org.firstinspires.ftc.teamcode.helpers.controls.button.ButtonCtl;
import org.firstinspires.ftc.teamcode.helpers.controls.rumble.RumbleControls;
import org.firstinspires.ftc.teamcode.helpers.controls.trigger.TriggerCtl;
import org.firstinspires.ftc.teamcode.subsystems.arm.MainArmConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.arm.SetArmPosition;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawAngle;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawState;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawTwist;

public class SampleMap extends ControlMap {
    GlobalMap globalMap;
    CommandScheduler cs;

    Follower f;
    RumbleControls rc;

    ElapsedTime retractTimer = new ElapsedTime();

    MainArmConfiguration.SAMPLE_SCORE_HEIGHT armState = MainArmConfiguration.SAMPLE_SCORE_HEIGHT.HIGH_BASKET;


    public SampleMap(DriverControls driverControls, CommandScheduler cs, GlobalMap globalMap) {
        super(driverControls);
        this.cs = cs;
        this.f = globalMap.f;
        this.rc = globalMap.rc;
        this.globalMap = globalMap;
    }

    @Override
    public void initialize() {
        gp.add(new ButtonCtl(GamepadKeys.Button.DPAD_LEFT, this::toggleArmLowState));

        gp.add(new ButtonCtl(GamepadKeys.Button.LEFT_BUMPER, this::subGrab));
        gp.add(new ButtonCtl(GamepadKeys.Button.RIGHT_BUMPER, this::deposit));

        gp.add(new TriggerCtl(GamepadKeys.Trigger.RIGHT_TRIGGER, (Double a) -> {
            if (a > 0.3) retractArm();
        }));
    }


    //
    // ARM OPS
    //
    private void retractArm() {
        if (retractTimer.milliseconds() > 800) {
            cs.schedule(new SetArmPosition().retract());
            retractTimer.reset();
        }
    }

    private void toggleArmLowState() {
        if (armState == MainArmConfiguration.SAMPLE_SCORE_HEIGHT.HIGH_BASKET) {
            armState = MainArmConfiguration.SAMPLE_SCORE_HEIGHT.LOW_BASKET;
        } else {
            armState = MainArmConfiguration.SAMPLE_SCORE_HEIGHT.HIGH_BASKET;
        }
    }

    // TODO - HANG

    private void subGrab() {
        globalMap.followerActive = true;
        f.holdPoint(f.getPose());

        cs.schedule(
                new SequentialCommandGroup(
                        new SubmersibleGrabV2(f, globalMap.reader, rc),
                        new WaitCommand(230),
                        new SetClawState(ClawConfiguration.GripperState.CLOSED),
                        new WaitCommand(150),
                        new SetArmPosition().retract()
                )
        );
    }

    private void deposit() {
        globalMap.followerActive = true;
        cs.schedule(
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new SetClawTwist(ClawConfiguration.HorizontalRotation.NORMAL),
                                new SetClawAngle(ClawConfiguration.VerticalRotation.UP),
                                new SequentialCommandGroup(
                                        new SetArmPosition().retract(),
                                        new SetArmPosition().scoreSample(armState)
                                ),
                                new SequentialCommandGroup(
                                        new FollowPath(f, bezierPath(f.getPose(), SUB_GRAB_0, BUCKET_HIGH_SCORE_POSE)
                                                .setLinearHeadingInterpolation(SUB_GRAB.getHeading(), BUCKET_HIGH_SCORE_POSE.getHeading()).build()
                                        ),
                                        new InstantCommand() {
                                            @Override
                                            public void run() {
                                                globalMap.followerActive = false;
                                                rc.singleBlip();
                                            }
                                        }
                                )
                        )

                )
        );
    }
}
