package org.firstinspires.ftc.teamcode.subsystems.arm.commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideSubsystem;

public class ResetSlideCoeffs extends InstantCommand {
    public ResetSlideCoeffs(){
        VLRSubsystem.getInstance(ArmSlideSubsystem.class).resetMotionProfileCoeffs();
    }
}
