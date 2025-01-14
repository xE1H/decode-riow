package org.firstinspires.ftc.teamcode.subsystems.vision;

public class Point3d {
    double x;
    double y;
    double z;

    Point3d(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    @Override
    public String toString() {
        return String.format("(%.2f, %.2f, %.2f)", x, y, z);
    }
}