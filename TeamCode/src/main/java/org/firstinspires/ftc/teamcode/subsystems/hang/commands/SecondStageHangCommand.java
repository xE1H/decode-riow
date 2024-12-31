package org.firstinspires.ftc.teamcode.subsystems.hang.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmState;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.CustomConditionalCommand;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.MoveArmInToRobot;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.ResetMaxSlideVelocity;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.SetMaxSlideVelocity;
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
                        () -> ArmState.get() == ArmState.State.INTAKE
                ),

                new SetClawAngle(ClawConfiguration.TargetAngle.UP),
                new SetRotatorAngle(92),
                new WaitUntilCommand(() -> VLRSubsystem.getInstance(ArmRotatorSubsystem.class).getAngleDegrees() >= 60),
                new SetSlideExtension(0.85),
                new WaitUntilCommand(()-> VLRSubsystem.getInstance(ArmSlideSubsystem.class).reachedTargetPosition()),
                new WaitUntilCommand(gamepadCondition),
                new SetMaxSlideVelocity(200),
                new SetSlideExtension(0.75),
                new WaitUntilCommand(()-> VLRSubsystem.getInstance(ArmSlideSubsystem.class).reachedTargetPosition()),
                new ResetMaxSlideVelocity()
        );

        addRequirements(VLRSubsystem.getInstance(ArmRotatorSubsystem.class), VLRSubsystem.getInstance(ArmSlideSubsystem.class));
    }
}
