package org.firstinspires.ftc.teamcode.subsystems.arm.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorConfiguration.ArmState;
import org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration.TargetAngle;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration.TargetState;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawAngle;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawState;

public class SetArmState_Intake extends SequentialCommandGroup {

    public SetArmState_Intake() {
        ArmRotatorSubsystem arm = VLRSubsystem.getInstance(ArmRotatorSubsystem.class);
        ArmSlideSubsystem slides = VLRSubsystem.getInstance(ArmSlideSubsystem.class);

        if (arm.getArmState() != ArmState.INTAKE) {
            addCommands(
                    new CustomConditionalCommand(
                            new SetArmState_InRobot(),
                            () -> arm.getArmState() == ArmState.DEPOSIT
                    ),

                    new SetClawAngle(TargetAngle.UP),
                    new SetSlideExtension(ArmSlideConfiguration.TargetPosition.INTAKE),
                    new WaitUntilCommand(slides::reachedTargetPosition),
                    new SetClawAngle(TargetAngle.DEPOSIT),
                    new SetClawState(TargetState.OPEN),
                    new SetCurrentState(ArmState.INTAKE)
            );
            addRequirements(arm, slides);
        }
    }
}