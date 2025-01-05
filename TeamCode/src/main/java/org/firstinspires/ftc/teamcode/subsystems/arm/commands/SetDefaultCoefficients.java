package org.firstinspires.ftc.teamcode.subsystems.arm.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideSubsystem;

public class SetDefaultCoefficients extends ParallelCommandGroup {
    public SetDefaultCoefficients(){
        addCommands(
                new InstantCommand(()-> VLRSubsystem.getInstance(ArmSlideSubsystem.class).setDefaultCoefficients()),
                new InstantCommand(()-> VLRSubsystem.getInstance(ArmRotatorSubsystem.class).setDefaultCoefficients())
        );
    }
}
