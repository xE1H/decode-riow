package org.firstinspires.ftc.teamcode.subsystems.arm.commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideSubsystem;

public class SetSlideExtension extends InstantCommand {

    public SetSlideExtension(ArmSlideConfiguration.TargetPosition extension) {
        super(() -> VLRSubsystem.getInstance(ArmSlideSubsystem.class).setTargetPosition(extension));
    }
}
