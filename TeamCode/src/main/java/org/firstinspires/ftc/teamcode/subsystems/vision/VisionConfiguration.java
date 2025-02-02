package org.firstinspires.ftc.teamcode.subsystems.vision;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Scalar;

@Config
public class VisionConfiguration {
    public static String CAMERA_NAME = "Webcam 1";
    /*
    <?xml version="1.0" encoding="UTF-8"?>
    <calibrations>
        <camera cameraMaker="NA" cameraModel="NA" lense="NA" fx="985.060059976" fy="985.060059976"
        cx="655.17501135" cy="377.68001436" k1="0.164332494036" k2="-0.971981355491"
        p1="-0.00343593218842" p2="-7.43545604089e-05" k3="1.01105277036" skew="0" name="NA NA (NA)"/>
    </calibrations>
     */
    public static Size RESOLUTION = new Size(1280, 720);

    public static double FX = 985.060059976;
    public static double FY = 985.060059976;
    public static double CX = 655.17501135;
    public static double CY = 377.68001436;

    public static double K1 = 0.164332494036;
    public static double K2 = -0.971981355491;
    public static double P1 = -0.00343593218842;
    public static double P2 = -7.43545604089e-05;
    public static double K3 = 1.01105277036;

    public static double MIN_AREA_BOUND = 500;
    public static double MAX_AREA_BOUND = 50000;

    public static Scalar[] LOWER_COLOR_BOUNDS = {
            //new Scalar(0, 85, 85),    // Red lower
            //new Scalar(100, 85, 85),  // Blue lower
            new Scalar(16, 85, 85)    // Yellow lower
    };
    public static Scalar[] UPPER_COLOR_BOUNDS = {
            //new Scalar(10, 255, 255),   // Red upper
            //new Scalar(130, 255, 255),  // Blue upper
            new Scalar(30, 255, 255)    // Yellow upper
    };

//    public static double CAMERA_HEIGHT = 7.5984; // inches from the ground 19.3 CM
//    public static double CAMERA_ANGLE_H = Math.PI / 12; // radians downward tilt from horizontal 15DEG
//    public static double CAMERA_ANGLE_X = Math.PI / 18; // radians left/right angle from straight 10DEG
//    public static double CAMERA_ROLL = -Math.PI / 2; // radians roll angle 90DEG
//    public static double CAMERA_BACKWARD_OFFSET = 4.3307; // inches from the front of the robot 11CM
//    public static double CAMERA_LEFT_OFFSET = 3.7401; // inches right from robot center 9.5CM

    public static double POS_Z = 7.5984; // inches from the ground 19.3 CM
    public static double POS_X = 3.7401; // inches right from robot center 9.5CM
    public static double POS_Y = -4.3307; // inches from the front of the robot 11CM

    public static double LEFT_ANGLE = 13.2; // radians
    public static double DOWN_ANGLE = -100; // radians

    public static double MAX_REL_Y = 12; // inches todo
//
//    public static double ROT_Y = -Math.PI / 2; // radians roll angle 90DEG
//    public static double ROT_Z = Math.PI / 18; // radians left/right angle from straight 10DEG
//    public static double ROT_X = Math.PI / 12; // radians downward tilt from horizontal 15DEG

    // azimuth angle is a = pi/2 + pi/18
    // elevation angle is -pi/12
}
