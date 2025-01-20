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
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.SetRotatorAngle;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.SetSlideExtension;
import org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration.VerticalRotation;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawAngle;

public class ScoreSampleHigh extends CustomConditionalCommand {
    {
        // This needs to be here, since addRequirements needs to be called BEFORE the command is
        // able to run. The running may happen instantly (on super() being called), or at any point
        // in the future, so it's best to call addRequirements as soon as possible, in this case
        // before the constructor ever runs.
        addRequirements(VLRSubsystem.getInstance(ArmRotatorSubsystem.class), VLRSubsystem.getInstance(ArmSlideSubsystem.class));
    }

    public ScoreSampleHigh() {
        super(new SequentialCommandGroup(
                        new CustomConditionalCommand(
                                new RetractArm(),
                                () -> !ArmState.isCurrentState(ArmState.State.SCORE_SAMPLE_HIGH, ArmState.State.IN_ROBOT)
                        ),
                        new SetIsArmMoving(),

                        new SetRotatorAngle(ArmRotatorConfiguration.TargetAngle.SCORE_SAMPLE_HIGH),
                        new WaitUntilCommand(() -> VLRSubsystem.getInstance(ArmRotatorSubsystem.class).getAngleDegrees() >= 30),
                        new SetClawAngle(VerticalRotation.DOWN),

                        new WaitUntilCommand(VLRSubsystem.getInstance(ArmRotatorSubsystem.class)::reachedTargetPosition),
                        new SetSlideExtension(ArmSlideConfiguration.TargetPosition.SCORE_BUCKET_HIGH),

                        new WaitUntilCommand(VLRSubsystem.getInstance(ArmSlideSubsystem.class)::reachedTargetPosition),
                        new SetClawAngle(VerticalRotation.DEPOSIT),
                        new SetCurrentArmState(ArmState.State.SCORE_SAMPLE_HIGH)
                ),
                () -> (ArmState.get() != ArmState.State.SCORE_SAMPLE_HIGH && !ArmState.isMoving()) || ArmOverrideState.get()
        );
    }
}