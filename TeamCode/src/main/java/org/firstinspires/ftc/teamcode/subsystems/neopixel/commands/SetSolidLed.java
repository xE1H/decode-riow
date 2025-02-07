package org.firstinspires.ftc.teamcode.subsystems.neopixel.commands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.neopixel.NeoPixelConfiguration;

public class SetSolidLed extends ParallelCommandGroup {
    public SetSolidLed(NeoPixelConfiguration.Colour color) {
        addCommands(
                new SetBrightness(1),
                new SetColour(color),
                new SetEffect(NeoPixelConfiguration.Effect.SOLID_COLOR),
                new SetEffectTime(1)
        );
    }
}
