package org.firstinspires.ftc.teamcode.subsystems.claw.commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;

public class SetClawTwist extends InstantCommand {

    public SetClawTwist(ClawConfiguration.HorizontalRotation twist) {
        super(() -> VLRSubsystem.getInstance(ClawSubsystem.class).setHorizontalRotation(twist));
    }

    public SetClawTwist(double twist) {
        super(() -> VLRSubsystem.getInstance(ClawSubsystem.class).setHorizontalRotation(twist));
    }
}
