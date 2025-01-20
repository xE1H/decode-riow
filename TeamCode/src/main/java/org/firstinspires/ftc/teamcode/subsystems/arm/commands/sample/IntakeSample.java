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
                        new CustomConditionalCommand(
                                new RetractArm(),
                                () -> !ArmState.isCurrentState(ArmState.State.INTAKE_SAMPLE, ArmState.State.IN_ROBOT)
                        ),
                        new SetIsArmMoving(),

                        new SetClawAngle(VerticalRotation.UP),
                        new SetSlideExtension(extension),
                        new WaitUntilCommand(VLRSubsystem.getInstance(ArmRotatorSubsystem.class)::reachedTargetPosition),
                        new SetClawAngle(VerticalRotation.DEPOSIT),
                        new SetClawState(GripperState.CLOSED),
                        new SetCurrentArmState(ArmState.State.INTAKE_SAMPLE)
                ),
                () -> (!ArmState.isCurrentState(ArmState.State.INTAKE_SAMPLE) && !ArmState.isMoving()) || ArmOverrideState.get()
        );
    }

    public IntakeSample() {
        this(ArmSlideConfiguration.TargetPosition.INTAKE_SAMPLE.extension);
    }

}