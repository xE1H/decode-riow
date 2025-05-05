package org.firstinspires.ftc.teamcode.helpers.utils;

public class Point {
    private final double magnitude;
    private final double angleDegrees;

    public Point (double magnitude, double angleDegrees){
        this.angleDegrees = angleDegrees;
        this.magnitude = magnitude;
    }

    public double magnitude(){return magnitude;}

    public double angleDegrees(){return angleDegrees;}

    public double angleRads(){return Math.toRadians(angleDegrees);}

    public double getX(){return magnitude * Math.cos(angleRads());}

    public double getY(){return magnitude * Math.sin(angleRads());}


    public boolean equals(Point point){
        return point.angleDegrees == this.angleDegrees && point.magnitude == this.magnitude;
    }
}
