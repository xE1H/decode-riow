package org.firstinspires.ftc.teamcode.subsystems.arm.commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorSubsystem;

public class SetCurrentState extends InstantCommand {

    public SetCurrentState(ArmRotatorConfiguration.ArmState state) {
        super(() -> VLRSubsystem.getInstance(ArmRotatorSubsystem.class).setArmState(state));
    }
}
