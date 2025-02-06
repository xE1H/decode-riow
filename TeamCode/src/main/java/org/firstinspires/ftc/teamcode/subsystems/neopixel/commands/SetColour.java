package org.firstinspires.ftc.teamcode.subsystems.neopixel.commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.neopixel.NeoPixelConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.neopixel.NeoPixelSubsystem;

public class SetColour extends InstantCommand {
    public SetColour(NeoPixelConfiguration.Colour colour){
        super(()-> VLRSubsystem.getInstance(NeoPixelSubsystem.class).setColor(colour));
    }
}
