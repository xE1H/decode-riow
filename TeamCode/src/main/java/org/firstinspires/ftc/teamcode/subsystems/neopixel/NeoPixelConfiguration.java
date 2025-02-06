package org.firstinspires.ftc.teamcode.subsystems.neopixel;

import com.acmerobotics.dashboard.config.Config;


public interface NeoPixelConfiguration {
    public int ledCount = 20;
    String NeoPixelName = "NeoPixel";

    public enum Effect
    {
        SOLID_COLOR,
        BREATHE,
        BLINK,
        CHASE_FORWARD,
        CHASE_BACKWARD,
        CRAZYMODE;

    }

    public enum Colour {
        RED(255, 0,  0),
        GREEN(0, 255, 0),
        BLUE(0, 0, 255),
        YELLOW(255, 255, 0),
        CYAN(0, 255, 255),
        PURPLE(255, 0, 255),
        WHITE (255, 255, 255);
        public final int r, g, b;

        Colour(int r, int g, int b) {
            this.r = r;
            this.g = g;
            this.b = b;
        }

    }

}
