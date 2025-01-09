package org.firstinspires.ftc.teamcode.helpers.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawState;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;

public class GrabSample extends SequentialCommandGroup {

    public GrabSample() {
        ClawSubsystem claw = VLRSubsystem.getInstance(ClawSubsystem.class);
        addCommands(
                new SetClawState(ClawConfiguration.TargetState.CLOSED),
                new SequentialCommandGroup(
                        new WaitCommand(200),
                        new SetClawState(ClawConfiguration.TargetState.CLOSED)));
        addRequirements(claw);
    }
}
