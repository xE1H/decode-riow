package org.firstinspires.ftc.teamcode.subsystems.arm.slide;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ArmSlideConfiguration {
    public static String MOTOR_NAME_0 = "MotorArm1";
    public static String MOTOR_NAME_1 = "MotorArm2";
    public static String MOTOR_NAME_2 = "MotorArm3";
    public static String ENCODER_NAME = "MotorLeftBack";

    public static String LIMIT_SW_NAME = "SlideLimit";

    public static double ACCELERATION = 150;
    public static double DECELERATION = 120;
    public static double MAX_VELOCITY = 50;
    public static double FEEDBACK_PROPORTIONAL_GAIN = 0.1;
    public static double FEEDBACK_INTEGRAL_GAIN = 0;
    public static double FEEDBACK_DERIVATIVE_GAIN = 0.005;
    public static double FEED_FORWARD_GAIN = 0;
    public static double VELOCITY_GAIN = 0;
    public static double ACCELERATION_GAIN = 0;

    public static double ACCELERATION_HANG = 70;
    public static double DECELERATION_HANG = 50;
    public static double MAX_VELOCITY_HANG = 20;
    public static double FEEDBACK_PROPORTIONAL_GAIN_HANG = 0.6;
    public static double FEEDBACK_DERIVATIVE_GAIN_HANG = 0.013;
    public static double FEEDBACK_INTEGRAL_GAIN_HANG = 0.00001;
    public static double FEED_FORWARD_GAIN_HANG = -0.25;
    public static double VELOCITY_GAIN_HANG = 0;
    public static double ACCELERATION_GAIN_HANG = 0;

    public static double FEEDBACK_PROPORTIONAL_GAIN_HOLD_POINT = 0;
    public static double FEEDBACK_DERIVATIVE_GAIN_HOLD_POINT = 0;
    public static double FEEDBACK_INTEGRAL_GAIN_HOLD_POINT = 0;

    public static double CREEP = 2;
    public static double ERROR_MARGIN = 0.3;

    public static double MIN_POSITION = 0;
    public static double HORIZONTAL_EXTENSION_LIMIT = 15;
    public static double MAX_POSITION = 23; //ROTATIONS, NOT TICKS
    public static double MAX_EXTENSION_CM = 0;


}
