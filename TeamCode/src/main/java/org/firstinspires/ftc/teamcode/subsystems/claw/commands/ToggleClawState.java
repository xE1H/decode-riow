package org.firstinspires.ftc.teamcode.subsystems.claw.commands;

import com.arcrobotics.ftclib.command.ConditionalCommand;

import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;

public class ToggleClawState extends ConditionalCommand {
    public ToggleClawState(){
        super(new SetClawState(ClawConfiguration.GripperState.OPEN),
              new SetClawState(ClawConfiguration.GripperState.CLOSED),
              ()-> VLRSubsystem.getInstance(ClawSubsystem.class).getClawState() == ClawConfiguration.GripperState.CLOSED);
    }
}
