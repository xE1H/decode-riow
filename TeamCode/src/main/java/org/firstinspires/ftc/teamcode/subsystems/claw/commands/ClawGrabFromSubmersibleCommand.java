package org.firstinspires.ftc.teamcode.subsystems.claw.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.helpers.commands.GrabSample;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.commands.SetRotatorAngle;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;

public class ClawGrabFromSubmersibleCommand extends SequentialCommandGroup {
    public ClawGrabFromSubmersibleCommand() {
        ClawSubsystem claw = VLRSubsystem.getInstance(ClawSubsystem.class);
        ArmRotatorSubsystem arm = VLRSubsystem.getInstance(ArmRotatorSubsystem.class);

        if (arm.getRotatorState() == ArmRotatorConfiguration.RotatorState.PRE_INTAKE) {
            if (claw.getClawState() == ClawSubsystem.ClawState.OPEN) {
                claw.setClawState(ClawSubsystem.ClawState.CLOSED);
                addCommands(
                        new SetRotatorAngle(ArmRotatorConfiguration.TargetAngle.DOWN),
                        new GrabSample(),
                        new SetRotatorAngle(ArmRotatorConfiguration.TargetAngle.INTAKE)
                );
            } else {
                claw.setClawState(ClawSubsystem.ClawState.OPEN);
                addCommands(
                        new SetRotatorAngle(ArmRotatorConfiguration.TargetAngle.DOWN),
                        new SetClawState(ClawConfiguration.TargetState.OPEN),
                        new SetRotatorAngle(ArmRotatorConfiguration.TargetAngle.INTAKE)
                );
            }
        } else if (arm.getRotatorState() == ArmRotatorConfiguration.RotatorState.FIRST_BASKET) {
            if (claw.getClawState() == ClawSubsystem.ClawState.OPEN) {
                claw.setClawState(ClawSubsystem.ClawState.CLOSED);
                addCommands(
                        new SetClawState(ClawConfiguration.TargetState.CLOSED_NORMAL)
                );
            } else {
                claw.setClawState(ClawSubsystem.ClawState.OPEN);
                addCommands(
                        new SetClawState(ClawConfiguration.TargetState.OPEN)
                );
            }
        }
    }
}
