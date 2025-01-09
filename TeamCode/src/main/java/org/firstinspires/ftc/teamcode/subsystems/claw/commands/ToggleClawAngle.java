package org.firstinspires.ftc.teamcode.subsystems.claw.commands;

import com.arcrobotics.ftclib.command.ConditionalCommand;

import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;

public class ToggleClawAngle extends ConditionalCommand {
    public ToggleClawAngle(){
        super(new SetClawAngle(ClawConfiguration.TargetAngle.DOWN),
              new SetClawAngle(ClawConfiguration.TargetAngle.UP),
              ()-> VLRSubsystem.getInstance(ClawSubsystem.class).getTargetAngle() == ClawConfiguration.TargetAngle.UP);
    }
}
