package org.firstinspires.ftc.teamcode.subsystems.arm.commands.sample;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmOverrideState;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmState;
import org.firstinspires.ftc.teamcode.helpers.commands.CustomConditionalCommand;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.RetractArm;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.SetCurrentArmState;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.SetIsArmMoving;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.SetSlideExtension;
import org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration.VerticalRotation;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration.GripperState;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawAngle;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawState;
import org.firstinspires.ftc.teamcode.subsystems.neopixel.NeoPixelConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.neopixel.commands.SetColour;
import org.firstinspires.ftc.teamcode.subsystems.neopixel.commands.SetEffect;

public class IntakeSample extends CustomConditionalCommand {
    {
        // This needs to be here, since addRequirements needs to be called BEFORE the command is
        // able to run. The running may happen instantly (on super() being called), or at any point
        // in the future, so it's best to call addRequirements as soon as possible, in this case
        // before the constructor ever runs.
        addRequirements(VLRSubsystem.getInstance(ArmRotatorSubsystem.class), VLRSubsystem.getInstance(ArmSlideSubsystem.class));
    }

    public IntakeSample(double extension) {
        super(new SequentialCommandGroup(
                        new SetIsArmMoving(),
                        new CustomConditionalCommand(
                                new RetractArm(),
                                () -> !ArmState.isCurrentState(ArmState.State.SAMPLE_INTAKE, ArmState.State.IN_ROBOT)
                        ),

                        new SetEffect(NeoPixelConfiguration.Effect.CHASE_FORWARD),
                        new SetColour(NeoPixelConfiguration.Colour.YELLOW),


                        new SetClawAngle(VerticalRotation.UP),
                        new SetSlideExtension(extension),
                        new WaitUntilCommand(VLRSubsystem.getInstance(ArmSlideSubsystem.class)::reachedTargetPosition),
                        new SetClawAngle(VerticalRotation.DEPOSIT),
                        new SetClawState(GripperState.CLOSED),
                        new SetCurrentArmState(ArmState.State.SAMPLE_INTAKE)
                ),
                () -> (!ArmState.isCurrentState(ArmState.State.SAMPLE_INTAKE) && !ArmState.isMoving()) || ArmOverrideState.get()
        );

        System.out.println(ArmState.get());
        System.out.println(ArmState.isMoving());

    }

    public IntakeSample() {
        this(ArmSlideConfiguration.TargetPosition.INTAKE_SAMPLE.extension);
    }

}