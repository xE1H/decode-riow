package org.firstinspires.ftc.teamcode.subsystems.neopixel.commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.neopixel.NeoPixelSubsystem;

public class SetTrailLength extends InstantCommand {
    public SetTrailLength(int trailLength){
        super(()-> VLRSubsystem.getInstance(NeoPixelSubsystem.class).setTrailLength(trailLength));
    }
}
