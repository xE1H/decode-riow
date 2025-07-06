package org.firstinspires.ftc.teamcode.subsystems.arm.slide;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ArmSlideConfiguration {
    public static String MOTOR_NAME_0 = "MotorArm1";
    public static String MOTOR_NAME_1 = "MotorArm2";
    public static String MOTOR_NAME_2 = "MotorArm3";
    public static String ENCODER_NAME = "MotorRightBack";

    public static String LIMIT_SW_NAME = "SlideLimit";

    public static double ACCELERATION = 340;
    public static double DECELERATION = 220;
    public static double MAX_VELOCITY = 108;
    public static double FEEDBACK_PROPORTIONAL_GAIN = 0.2;
    public static double FEEDBACK_INTEGRAL_GAIN = 0.0001;
    public static double FEEDBACK_DERIVATIVE_GAIN = 0.0125;
    public static double FEED_FORWARD_GAIN = 0.06;
    public static double VELOCITY_GAIN = 0.024;
    public static double ACCELERATION_GAIN = 0.0002;

    public static double ACCELERATION_HANG = 60;
    public static double DECELERATION_HANG = 50;
    public static double MAX_VELOCITY_HANG = 11;
    public static double FEEDBACK_PROPORTIONAL_GAIN_HANG = 0.275;
    public static double FEEDBACK_DERIVATIVE_GAIN_HANG = 0.03;
    public static double FEEDBACK_INTEGRAL_GAIN_HANG = 0.000;
    public static double FEED_FORWARD_GAIN_HANG = -0.15;
    public static double VELOCITY_GAIN_HANG = 0.02;
    public static double ACCELERATION_GAIN_HANG = 0.0002;

    public static double FEEDBACK_PROPORTIONAL_GAIN_HOLD_POINT = 0.65;
    public static double FEEDBACK_DERIVATIVE_GAIN_HOLD_POINT = 0.02;
    public static double FEEDBACK_INTEGRAL_GAIN_HOLD_POINT = 0.01;

    public static double CREEP = 3;
    public static double ERROR_MARGIN = 1.2;

    public static double MIN_POSITION = 0;
    public static double HORIZONTAL_EXTENSION_LIMIT = 15;
    public static double MAX_POSITION = 25.3; //ROTATIONS, NOT TICKS
    public static double MAX_EXTENSION_CM = 73.65;


}
