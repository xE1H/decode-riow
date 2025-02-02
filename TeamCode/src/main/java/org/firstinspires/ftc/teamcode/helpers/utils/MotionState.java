package org.firstinspires.ftc.teamcode.helpers.utils;

public class MotionState {
    
    public double position;
    public double velocity;
    public double acceleration;
    public double jerk;

    public MotionState(double position, double velocity, double acceleration) {
        this.position = position;
        this.velocity = velocity;
        this.acceleration = acceleration;
    }


    public MotionState(double position, double velocity, double acceleration, double jerk) {
        this.position = position;
        this.velocity = velocity;
        this.acceleration = acceleration;
        this.jerk = jerk;
    }
}
