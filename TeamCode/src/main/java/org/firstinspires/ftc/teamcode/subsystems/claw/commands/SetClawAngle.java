package org.firstinspires.ftc.teamcode.subsystems.claw.commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;

public class SetClawAngle extends InstantCommand {

    public SetClawAngle(ClawConfiguration.VerticalRotation angle) {
        super(() -> VLRSubsystem.getInstance(ClawSubsystem.class).setTargetAngle(angle));
    }

    public SetClawAngle(double angle){
        super(() -> VLRSubsystem.getInstance(ClawSubsystem.class).setTargetAngle(angle));
    }
}
