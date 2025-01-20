package org.firstinspires.ftc.teamcode.subsystems.claw.commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;

public class SetClawState extends InstantCommand {

    public SetClawState(ClawConfiguration.GripperState state) {
        super(() -> VLRSubsystem.getInstance(ClawSubsystem.class).setTargetState(state));
    }
}
