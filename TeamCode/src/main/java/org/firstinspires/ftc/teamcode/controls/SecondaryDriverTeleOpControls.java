package org.firstinspires.ftc.teamcode.controls;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.helpers.controls.DriverControls;
import org.firstinspires.ftc.teamcode.helpers.controls.button.ButtonCtl;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmOverrideState;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmState;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.ResetRotatorMotor;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.sample.ScoreSample;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.RetractArm;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.sample.IntakeSample;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.specimen.HangSpecimen;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.specimen.IntakeSpecimenTeleOp;
import org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.ToggleClawAngle;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.ToggleClawState;

/**
 * Abstraction for secondary driver controls. All controls will be defined here.
 * For this to work well, all subsystems will be defined as singletons.
 */
public class SecondaryDriverTeleOpControls extends DriverControls {
    ClawSubsystem claw;
    ArmSlideSubsystem slide;

    long lastInterval = System.nanoTime();

    public SecondaryDriverTeleOpControls(Gamepad gamepad) {
        super(new GamepadEx(gamepad));

        CommandScheduler cs = CommandScheduler.getInstance();

        claw = VLRSubsystem.getInstance(ClawSubsystem.class);
        slide = VLRSubsystem.getInstance(ArmSlideSubsystem.class);

        add(new ButtonCtl(CROSS, ButtonCtl.Trigger.WAS_JUST_PRESSED, true, (Boolean a) -> cs.schedule(new IntakeSample())));
        add(new ButtonCtl(SQUARE, ButtonCtl.Trigger.WAS_JUST_PRESSED, true, (Boolean b) -> cs.schedule(new RetractArm())));
        add(new ButtonCtl(TRIANGLE, ButtonCtl.Trigger.WAS_JUST_PRESSED, true, (Boolean c) -> cs.schedule(new ScoreSample(107))));
        add(new ButtonCtl(CIRCLE, ButtonCtl.Trigger.SIMPLE, false, ArmOverrideState::set));

        add(new ButtonCtl(GamepadKeys.Button.DPAD_DOWN, ButtonCtl.Trigger.WAS_JUST_PRESSED, true, (Boolean d) -> cs.schedule(new ToggleClawState())));
        add(new ButtonCtl(GamepadKeys.Button.DPAD_LEFT, ButtonCtl.Trigger.WAS_JUST_PRESSED, true, (Boolean e) -> cs.schedule(new ToggleClawAngle())));

    //    add(new ButtonCtl(GamepadKeys.Button.LEFT_BUMPER, ButtonCtl.Trigger.WAS_JUST_PRESSED, true, (Boolean f) -> cs.schedule(new ResetRotatorMotor())));

        add(new ButtonCtl(GamepadKeys.Button.RIGHT_BUMPER, ButtonCtl.Trigger.WAS_JUST_PRESSED, true, (Boolean g) -> cs.schedule (new HangSpecimen())));

        add(new ButtonCtl(GamepadKeys.Button.LEFT_BUMPER, ButtonCtl.Trigger.WAS_JUST_PRESSED, true, (Boolean h) -> cs.schedule(new IntakeSpecimenTeleOp())));

        addRightStickHandler((Double x, Double y) -> incrementClaw(y));
        addLeftStickHandler((Double x, Double y) -> incrementSlidePosition(x));

        addVibration(ArmOverrideState::get);
    }

    private void incrementClaw(double input) {
        if (VLRSubsystem.getInstance(ClawSubsystem.class) != null)
            VLRSubsystem.getInstance(ClawSubsystem.class).setHorizontalRotation(input);
    }

    private void incrementSlidePosition(double input) {
        System.out.printf("HANG SECOND: " + ArmState.isCurrentState(ArmState.State.HANG_SECOND_STAGE));
        System.out.printf("HANG INTAKE: " + (ArmState.isCurrentState(ArmState.State.SAMPLE_INTAKE) && !ArmState.isMoving() && !ArmOverrideState.get()));
        if (ArmState.isCurrentState(ArmState.State.HANG_SECOND_STAGE) || (ArmState.isCurrentState(ArmState.State.SAMPLE_INTAKE) && !ArmState.isMoving() && !ArmOverrideState.get())) {
            System.out.println("HANG: IF ACCESSES");
            slide.incrementTargetPosition(input * 0.5d / 1000000 * Math.abs(System.nanoTime() - lastInterval) * (ArmState.isCurrentState(ArmState.State.HANG_SECOND_STAGE) ? -0.5 : 0.5));
        }
        lastInterval = System.nanoTime();
    }
}
