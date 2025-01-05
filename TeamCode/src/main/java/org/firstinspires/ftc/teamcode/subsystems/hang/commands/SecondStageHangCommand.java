package org.firstinspires.ftc.teamcode.subsystems.hang.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmState;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.CustomConditionalCommand;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.MoveArmInToRobot;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.SetDefaultCoefficients;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.SetArmState;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.SetHangCoefficients;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.SetRotatorAngle;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.SetSlideExtension;
import org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawAngle;

import java.util.function.BooleanSupplier;

public class SecondStageHangCommand extends SequentialCommandGroup {
    public SecondStageHangCommand(BooleanSupplier gamepadCondition){
        addCommands(
                new CustomConditionalCommand(
                        new MoveArmInToRobot(),
                        () -> (ArmState.get() == ArmState.State.INTAKE || ArmState.get() == ArmState.State.DEPOSIT)
                ),

                new SetClawAngle(ClawConfiguration.TargetAngle.UP),
                new SetRotatorAngle(97),
                new WaitUntilCommand(() -> VLRSubsystem.getInstance(ArmRotatorSubsystem.class).getAngleDegrees() >= 60),
                new SetSlideExtension(0.942),
                new WaitUntilCommand(()-> VLRSubsystem.getInstance(ArmSlideSubsystem.class).reachedTargetPosition()),
                new SetArmState(ArmState.State.SECOND_STAGE_HANG),
                new WaitUntilCommand(gamepadCondition),
                new SetHangCoefficients(),
                new SetSlideExtension(0.735),
                new WaitCommand(500),
                new SetRotatorAngle(89),
                new WaitUntilCommand(gamepadCondition),
                new SetDefaultCoefficients(),
                new SetRotatorAngle(97.5),
                new SetSlideExtension(0.88)
            );

        addRequirements(VLRSubsystem.getInstance(ArmRotatorSubsystem.class), VLRSubsystem.getInstance(ArmSlideSubsystem.class));
    }
}
