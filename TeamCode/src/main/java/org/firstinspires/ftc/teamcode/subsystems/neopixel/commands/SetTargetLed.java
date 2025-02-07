package org.firstinspires.ftc.teamcode.subsystems.neopixel.commands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.neopixel.NeoPixelConfiguration;

public class SetTargetLed extends ParallelCommandGroup {
    public SetTargetLed(NeoPixelConfiguration.Colour color) {
        addCommands(
                new SetBrightness(1),
                new SetColour(color),
                new SetEffect(NeoPixelConfiguration.Effect.BLINK),
                new SetEffectTime(0.5)
        );
    }
}
