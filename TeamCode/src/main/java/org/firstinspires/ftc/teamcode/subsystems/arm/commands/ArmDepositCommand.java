package org.firstinspires.ftc.teamcode.subsystems.arm.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawAngle;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawState;

public class ArmDepositCommand extends SequentialCommandGroup {
    public ArmDepositCommand() {
        ArmRotatorSubsystem arm = VLRSubsystem.getInstance(ArmRotatorSubsystem.class);
        if (arm.getRotatorState() == ArmRotatorConfiguration.RotatorState.IN_ROBOT) {
            arm.setRotatorState(ArmRotatorConfiguration.RotatorState.FIRST_BASKET);
            addCommands(
                    new SetRotatorAngle(ArmRotatorConfiguration.TargetAngle.DEPOSIT),
                    new SetSlideExtension(ArmSlideConfiguration.TargetPosition.DEPOSIT),
                    new SetClawAngle(ClawConfiguration.TargetAngle.DEPOSIT)
            );
        } else if (arm.getRotatorState() == ArmRotatorConfiguration.RotatorState.FIRST_BASKET) {
            arm.setRotatorState(ArmRotatorConfiguration.RotatorState.IN_ROBOT);
            addCommands(
                    new SetClawAngle(ClawConfiguration.TargetAngle.DEPOSIT),
                    new WaitCommand(300),
                    new SetSlideExtension(ArmSlideConfiguration.TargetPosition.RETRACTED),
                    new WaitCommand(3000),
                    new SetClawAngle(ClawConfiguration.TargetAngle.UP),
                    new SetClawState(ClawConfiguration.TargetState.CLOSED_NORMAL),
                    new WaitCommand(380),
                    new SetRotatorAngle(ArmRotatorConfiguration.TargetAngle.DOWN)
            );
        }
    }
}
