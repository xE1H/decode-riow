package org.firstinspires.ftc.teamcode.helpers.pedro;

public class MathUtils {
    public static double normalizeAngleDegrees(double angleDegrees){
        while (angleDegrees > 360){
            angleDegrees -= 360;
        }
        while (angleDegrees < -360){
            angleDegrees += 360;
        }
        if (angleDegrees > 180){
            angleDegrees = 360 - angleDegrees;
        }
        else if (angleDegrees < -180){
            angleDegrees = -360 + angleDegrees;
        }
        return angleDegrees;
    }

    public static double normalizeAngleRads(double angleRads){
        return Math.toRadians(normalizeAngleDegrees(Math.toDegrees(angleRads)));
    }
}
