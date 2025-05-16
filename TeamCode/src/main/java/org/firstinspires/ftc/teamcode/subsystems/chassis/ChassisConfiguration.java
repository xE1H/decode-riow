package org.firstinspires.ftc.teamcode.subsystems.chassis;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.pedropathing.localization.Pose;

@Config
public interface ChassisConfiguration {
    String MOTOR_LEFT_FRONT = "MotorLeftFront";
    String MOTOR_RIGHT_FRONT = "MotorRightFront";
    String MOTOR_LEFT_BACK = "MotorLeftBack";
    String MOTOR_RIGHT_BACK = "MotorRightBack";

    double DISTANCE_BETWEEN_ANGLED_SENSORS_MM = 0;
    Pose BUCKET_CORNER = new Pose(0, 0, 0);
    Vector2d OFFSET_FROM_SENSOR_MIDPOINT_TO_PEDRO_CENTER = new Vector2d();
}
