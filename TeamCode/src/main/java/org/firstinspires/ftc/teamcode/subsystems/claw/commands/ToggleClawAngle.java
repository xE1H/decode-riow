package org.firstinspires.ftc.teamcode.subsystems.claw.commands;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;

public class ToggleClawAngle extends ConditionalCommand {
    public ToggleClawAngle() {
        super(
                new SequentialCommandGroup(
                        new SetClawState(ClawConfiguration.GripperState.CLOSED),
                        new SetClawAngle(ClawConfiguration.VerticalRotation.DOWN),
                        new WaitCommand(50),
                        new SetClawState(ClawConfiguration.GripperState.OPEN)
                ),
                new SequentialCommandGroup(
                        new SetClawState(ClawConfiguration.GripperState.CLOSED),
                        new WaitCommand(200),
                        new SetClawAngle(ClawConfiguration.VerticalRotation.UP)
                ),
                () -> VLRSubsystem.getInstance(ClawSubsystem.class).getTargetAngle() != ClawConfiguration.VerticalRotation.DOWN)
        ;
    }
}
