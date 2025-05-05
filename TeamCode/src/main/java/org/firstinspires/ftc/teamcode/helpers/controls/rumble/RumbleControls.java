package org.firstinspires.ftc.teamcode.helpers.controls.rumble;

import com.qualcomm.robotcore.hardware.Gamepad;

public class RumbleControls {

    private final Gamepad gamepad;

    public RumbleControls(Gamepad gamepad) {
        this.gamepad = gamepad;
    }

    public void singleBlip() {
        gamepad.rumble(1, 1, 100);
    }

    public void rumbleBlips(int n) {
        gamepad.rumbleBlips(n);
    }

    public void doubleBlip() {
        Gamepad.RumbleEffect effect = new Gamepad.RumbleEffect.Builder()
                .addStep(1, 1, 70)
                .addStep(0, 0, 150)
                .addStep(1, 1, 70)
                .addStep(0, 0, 150)
                .build();

        gamepad.runRumbleEffect(effect);
    }

    public void doubleFade() {
        Gamepad.RumbleEffect.Builder effect = new Gamepad.RumbleEffect.Builder();

        for (int i = 0; i < 360; i++) {
            double sin = Math.sin((double) i * Math.PI / 180);
            double standard = (sin + 1) / 2;
            effect.addStep(standard, 1 - standard, 10);
        }


        gamepad.runRumbleEffect(effect.build());
    }
}
