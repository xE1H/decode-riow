package org.firstinspires.ftc.teamcode.subsystems.arm.commands;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawAngle;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawState;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawTwist;

public class ArmExtensionCommand extends SequentialCommandGroup {

    public ArmExtensionCommand() {
        ArmRotatorSubsystem arm = VLRSubsystem.getInstance(ArmRotatorSubsystem.class);
        if (arm.getRotatorState() == ArmRotatorConfiguration.RotatorState.IN_ROBOT) {
            arm.setRotatorState(ArmRotatorConfiguration.RotatorState.PRE_INTAKE);
            addCommands(
                    new SetClawTwist(ClawConfiguration.TargetTwist.NORMAL),
                    new SetRotatorAngle(ArmRotatorConfiguration.TargetAngle.INTAKE),
                    new SetSlideExtension(ArmSlideConfiguration.TargetPosition.INTAKE)
                    //new SetClawAngle(ClawConfiguration.TargetAngle.DOWN)
            );

        } else if (arm.getRotatorState() == ArmRotatorConfiguration.RotatorState.PRE_INTAKE) {
            arm.setRotatorState(ArmRotatorConfiguration.RotatorState.IN_ROBOT);
            addCommands(
                    new SetClawTwist(ClawConfiguration.TargetTwist.NORMAL),
                    new SetClawAngle(ClawConfiguration.TargetAngle.UP),
                    new SetClawState(ClawConfiguration.TargetState.CLOSED_NORMAL),
                    new SetSlideExtension(ArmSlideConfiguration.TargetPosition.RETRACTED),
                    new SetRotatorAngle(ArmRotatorConfiguration.TargetAngle.DOWN)
            );

        }
    }
}
