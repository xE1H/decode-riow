package org.firstinspires.ftc.teamcode.controlmaps;

import static org.firstinspires.ftc.teamcode.auto.sample.PointsSample.BUCKET_HIGH_SCORE_POSE;
import static org.firstinspires.ftc.teamcode.auto.sample.PointsSample.SUB_GRAB_0;
import static org.firstinspires.ftc.teamcode.helpers.pedro.PoseToPath.bezierPath;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.commands.FollowPath;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.auto.sample.SubmersibleGrabV2;
import org.firstinspires.ftc.teamcode.helpers.commands.CustomConditionalCommand;
import org.firstinspires.ftc.teamcode.helpers.commands.InstantCommand;
import org.firstinspires.ftc.teamcode.helpers.controls.DriverControls;
import org.firstinspires.ftc.teamcode.helpers.controls.button.ButtonCtl;
import org.firstinspires.ftc.teamcode.helpers.controls.rumble.RumbleControls;
import org.firstinspires.ftc.teamcode.helpers.controls.trigger.TriggerCtl;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.pedro.DriveToBucketTeleop;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmState;
import org.firstinspires.ftc.teamcode.subsystems.arm.MainArmConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.arm.ResetRotator;
import org.firstinspires.ftc.teamcode.subsystems.arm.ResetSlides;
import org.firstinspires.ftc.teamcode.subsystems.arm.SetArmPosition;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;

import java.util.Set;

public class SampleMap extends ControlMap {
    GlobalMap globalMap;
    CommandScheduler cs;

    Follower f;
    RumbleControls rc;

    ElapsedTime retractTimer = new ElapsedTime();
    MainArmConfiguration.SAMPLE_SCORE_HEIGHT armState = MainArmConfiguration.SAMPLE_SCORE_HEIGHT.HIGH_BASKET;

    boolean samplePickedUp = false;
    boolean depositCompleted = false;


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
        if (retractTimer.milliseconds() > 800 && depositCompleted) {
            cs.schedule(new SetArmPosition().retract().andThen(
                    new WaitCommand(100),
                    new ParallelCommandGroup(
                            new ResetSlides(),
                            new ResetRotator(),
                            new InstantCommand() {
                                @Override
                                public void run() {
                                    globalMap.followerActive = false;
                                    rc.singleBlip();
                            }}
                    ))
            );
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
                        new com.arcrobotics.ftclib.command.InstantCommand(()-> samplePickedUp = false),
                        new SubmersibleGrabV2(f, globalMap.reader, rc),

                        new ParallelCommandGroup(
                                new WaitCommand(200).andThen(
                                        new ConditionalCommand(
                                                new InstantCommand() {
                                                    @Override
                                                    public void run() {
                                                        deposit();
                                                    }
                                                },
                                                new SequentialCommandGroup(
                                                        new WaitCommand(200),
                                                        new ConditionalCommand(
                                                                new InstantCommand() {
                                                                    @Override
                                                                    public void run() {
                                                                        deposit();
                                                                    }
                                                                },
                                                                new InstantCommand() {
                                                                    @Override
                                                                    public void run() {
                                                                        globalMap.followerActive = false;
                                                                        rc.singleBlip();
                                                                    }
                                                                },
                                                                ()-> VLRSubsystem.getInstance(ClawSubsystem.class).isSamplePresent()
                                                        )

                                                ),
                                                ()-> VLRSubsystem.getInstance(ClawSubsystem.class).isSamplePresent()
                                        )
                                ),
                                new SetArmPosition().retract()
                        )
                )
        );
    }


    private void deposit() {
        globalMap.followerActive = true;
        depositCompleted = false;
        cs.schedule(
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new WaitUntilCommand(()-> getDistance(f.getPose(), BUCKET_HIGH_SCORE_POSE) < 50)
                                        .andThen(new SetArmPosition().scoreSample(armState)),
//                                new SetClawTwist(ClawConfiguration.HorizontalRotation.NORMAL),
//                                new SetClawAngle(ClawConfiguration.VerticalRotation.UP),
//                                new SequentialCommandGroup(
//                                        new SetArmPosition().retract(),
//                                        new SetArmPosition().scoreSample(armState)
//                                ),
                                new SequentialCommandGroup(
                                        new DriveToBucketTeleop(f),
                                        new InstantCommand() {
                                            @Override
                                            public void run() {
                                                globalMap.followerActive = false;
                                                rc.singleBlip();
                                            }
                                        }
                                )
                        ),
                        new com.arcrobotics.ftclib.command.InstantCommand(()-> depositCompleted = true)
                )
        );

    }

    private double getDistance(Pose start, Pose end){
        return Math.hypot(start.getX() - end.getX(), start.getY() - end.getY());
    }
}