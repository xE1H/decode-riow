package org.firstinspires.ftc.teamcode.subsystems.neopixel;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;

public class NeoPixelSubsystem extends VLRSubsystem<NeoPixelSubsystem> implements NeoPixelConfiguration {

    private NeoPixelDriver neoPixel;
    private double effectTime = 1;
    private Colour colour = Colour.RED;
    private Effect effect = Effect.SOLID_COLOR;
    private double brightness = 1.0;
    private int trailLength = 3; // Configurable trail length
    private ElapsedTime timer = new ElapsedTime();


    @Override
    protected void initialize(HardwareMap hardwareMap) {
        neoPixel = hardwareMap.get(NeoPixelDriver.class, NeoPixelName);
    }

    public void clear(){
        System.out.println("LED: CLEAR");
        this.effect = Effect.SOLID_COLOR;
        this.brightness = 0;
    }

    public void setColor(Colour colour) {
        if (this.colour == colour) return;
        this.colour = colour;
        timer.reset();
    }

    public void setTrailLength(int length) {
        this.trailLength = length;

    }

    public void setEffect(Effect effect) {
        if (this.effect == effect) return;
        this.effect = effect;
        timer.reset();
    }

    public void setEffectTime(double time) {
        if (this.effectTime == time) return;
        this.effectTime = time;
        timer.reset();
    }

    public void setBrightness(double brightness) {
        this.brightness = brightness;
    }

    @Override
    public void periodic() {
        switch (effect) {
            case SOLID_COLOR:
                for (int i = 1; i < NeoPixelConfiguration.ledCount + 1; i++) {
                    neoPixel.setColor(i, (int) (brightness * colour.r), (int) (brightness * colour.g), (int) (brightness * colour.b));
                }
                break;
            case BREATHE:
                double multiplier = (Math.sin(2 * Math.PI * timer.seconds() / effectTime) + 1) / 2.0;
                double sinR = brightness * colour.r * multiplier;
                double sinG = brightness * colour.g * multiplier;
                double sinB = brightness * colour.b * multiplier;
                for (int i = 1; i < NeoPixelConfiguration.ledCount + 1; i++) {
                    neoPixel.setColor(i, (int) sinR, (int) sinG, (int) sinB);
                }
                break;
            case BLINK:
                boolean active = (timer.seconds() % effectTime) > (effectTime / 2);
                double r = brightness * colour.r * (active ? 1 : 0);
                double g = brightness * colour.g * (active ? 1 : 0);
                double b = brightness * colour.b * (active ? 1 : 0);
                for (int i = 1; i < NeoPixelConfiguration.ledCount + 1; i++) {
                    neoPixel.setColor(i, (int) r, (int) g, (int) b);
                }
                break;
            case CHASE_FORWARD:
                double phaseForward = (timer.seconds() % effectTime) / effectTime;
                double headPositionForward = phaseForward * NeoPixelConfiguration.ledCount; // 0-based floating-point
                for (int i = 1; i <= NeoPixelConfiguration.ledCount; i++) {
                    double ledPos = i - 1; // Convert to 0-based
                    double delta = Math.abs(headPositionForward - ledPos);
                    delta = Math.min(delta, NeoPixelConfiguration.ledCount - delta); // Circular wrap
                    double intensity = Math.max(0, 1.0 - delta / trailLength);
                    neoPixel.setColor(i, (int)(brightness * colour.r * intensity), (int)(brightness * colour.g * intensity), (int)(brightness * colour.b * intensity));
                }
                break;
            case CHASE_BACKWARD:
                double phaseBackward = (timer.seconds() % effectTime) / effectTime;
                double headPositionBackward = NeoPixelConfiguration.ledCount - (phaseBackward * NeoPixelConfiguration.ledCount); // 0-based floating-point
                for (int i = 1; i <= NeoPixelConfiguration.ledCount; i++) {
                    double ledPos = i - 1; // Convert to 0-based
                    double delta = Math.abs(headPositionBackward - ledPos);
                    delta = Math.min(delta, NeoPixelConfiguration.ledCount - delta); // Circular wrap
                    double intensity = Math.max(0, 1.0 - delta / trailLength);
                    neoPixel.setColor(i, (int)(brightness * colour.r * intensity), (int)(brightness * colour.g * intensity), (int)(brightness * colour.b * intensity));
                }
                break;
            default:
                break;
        }
        neoPixel.show();
    }
}
