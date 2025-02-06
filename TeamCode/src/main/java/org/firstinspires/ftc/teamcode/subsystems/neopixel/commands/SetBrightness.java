package org.firstinspires.ftc.teamcode.subsystems.neopixel.commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.neopixel.NeoPixelSubsystem;

public class SetBrightness extends InstantCommand {
    public SetBrightness(double brightness){
        super(()-> VLRSubsystem.getInstance(NeoPixelSubsystem.class).setBrightness(brightness));
    }
}
