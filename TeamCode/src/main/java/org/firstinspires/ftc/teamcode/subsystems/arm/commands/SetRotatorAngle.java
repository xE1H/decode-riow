package org.firstinspires.ftc.teamcode.subsystems.arm.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorSubsystem;

public class SetRotatorAngle extends InstantCommand {

    public SetRotatorAngle(ArmRotatorConfiguration.TargetAngle angle) {
        super(() -> VLRSubsystem.getInstance(ArmRotatorSubsystem.class).setTargetAngle(angle));
    }

    public SetRotatorAngle(double degrees) {
        super(() -> VLRSubsystem.getInstance(ArmRotatorSubsystem.class).setTargetPosition(degrees));
    }
}
