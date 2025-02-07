package org.firstinspires.ftc.teamcode.subsystems.neopixel.commands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.neopixel.NeoPixelConfiguration;

public class SetLiftUpLed extends ParallelCommandGroup {
    public SetLiftUpLed(NeoPixelConfiguration.Colour color) {
        addCommands(
                new SetBrightness(1),
                new SetColour(color),
                new SetEffect(NeoPixelConfiguration.Effect.CHASE_FORWARD),
                new SetEffectTime(1)
        );
    }
}
