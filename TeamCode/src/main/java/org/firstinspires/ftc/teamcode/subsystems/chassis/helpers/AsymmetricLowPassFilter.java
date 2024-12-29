package org.firstinspires.ftc.teamcode.subsystems.chassis.helpers;

public class AsymmetricLowPassFilter {
    private double acceleration_a;
    private double deceleration_a;
    private double prevPower = 0;

    public AsymmetricLowPassFilter (double acceleration_a, double deceleration_a){
        this.acceleration_a = acceleration_a;
        this.deceleration_a = deceleration_a;
    }

    public double estimatePower(double power){
        double a;
        if (power > prevPower) a = acceleration_a;
        else a = deceleration_a;

        double output = a * (prevPower) + power * (1 - a);
        prevPower = power;

        return output;
    }
}
