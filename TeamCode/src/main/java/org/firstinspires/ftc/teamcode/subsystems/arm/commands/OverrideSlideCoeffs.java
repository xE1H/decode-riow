package org.firstinspires.ftc.teamcode.subsystems.arm.commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideSubsystem;

public class OverrideSlideCoeffs extends InstantCommand {
    public OverrideSlideCoeffs(double maxVelocity, double p, double i, double v, double a, double feedforward){
        VLRSubsystem.getInstance(ArmSlideSubsystem.class).overrideMotionProfileCoeffs(maxVelocity, p, i, v, a, feedforward);
    }
}
