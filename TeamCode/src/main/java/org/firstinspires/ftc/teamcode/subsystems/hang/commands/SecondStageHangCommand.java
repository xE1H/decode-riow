package org.firstinspires.ftc.teamcode.subsystems.hang.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmState;
import org.firstinspires.ftc.teamcode.helpers.commands.CustomConditionalCommand;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.RetractArm;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.SetCurrentArmState;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.SetHangCoefficients;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.SetRotatorAngle;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.SetSlideExtension;
import org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawAngle;
import org.firstinspires.ftc.teamcode.subsystems.hang.HangConfiguration;

import java.util.function.BooleanSupplier;

public class SecondStageHangCommand extends SequentialCommandGroup {
    public SecondStageHangCommand(BooleanSupplier gamepadCondition){
        addCommands(
                new CustomConditionalCommand(
                        new RetractArm(),
                        () -> (ArmState.get() != ArmState.State.SECOND_STAGE_HANG && ArmState.get() != ArmState.State.IN_ROBOT)
                ),

                new SetClawAngle(ClawConfiguration.TargetAngle.UP),
                new SetRotatorAngle(105),
                new WaitUntilCommand(() -> VLRSubsystem.getInstance(ArmRotatorSubsystem.class).getAngleDegrees() >= 60),
                new SetSlideExtension(0.942),
                new WaitUntilCommand(()-> VLRSubsystem.getInstance(ArmSlideSubsystem.class).reachedTargetPosition()),
                new SetCurrentArmState(ArmState.State.SECOND_STAGE_HANG),
                // hang off 2nd
                new WaitUntilCommand(gamepadCondition),
                new SetHangCoefficients(),
                new SetSlideExtension(0.5),
                new SetHangPosition(HangConfiguration.TargetPosition.HALF),
                new WaitCommand(500),
                new SetRotatorAngle(160),
                // release off 2nd, hang on 1st
                new WaitUntilCommand(gamepadCondition),
                new SetRotatorAngle(160),
                new SetHangPosition(HangConfiguration.TargetPosition.UP),
                new WaitCommand(500),
                //new SetDefaultCoefficients(),
                new SetSlideExtension(0.88)
            );

        addRequirements(VLRSubsystem.getInstance(ArmRotatorSubsystem.class), VLRSubsystem.getInstance(ArmSlideSubsystem.class));
    }
}
