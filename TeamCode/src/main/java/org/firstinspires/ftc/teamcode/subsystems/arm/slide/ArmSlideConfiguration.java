package org.firstinspires.ftc.teamcode.subsystems.arm.slide;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ArmSlideConfiguration {
    public static String MOTOR_NAME_0 = "MotorArm1";
    public static String MOTOR_NAME_1 = "MotorArm2";
    public static String MOTOR_NAME_2 = "MotorArm3";
    public static String ENCODER_NAME = "MotorRightBack";

    public static String LIMIT_SW_NAME = "SlideLimit";

//    public static double ACCELERATION_JERK = 700000;
//    public static double DECELERATION_JERK = 480000;
//    public static double MAX_VELOCITY = 3100;
//    public static double FEEDBACK_PROPORTIONAL_GAIN = 0.0255;
//    public static double FEEDBACK_INTEGRAL_GAIN = 0.000001;
//    public static double FEEDBACK_DERIVATIVE_GAIN = 0.0004;
//    public static double FEED_FORWARD_GAIN = 0.1;
//    public static double VELOCITY_GAIN = 0.0002;
//    public static double ACCELERATION_GAIN = 0.000016;


    public static double ACCELERATION = 26000;
    public static double DECELERATION = 16000;
    public static double MAX_VELOCITY = 2100;
    public static double FEEDBACK_PROPORTIONAL_GAIN = 0.0085;
    public static double FEEDBACK_INTEGRAL_GAIN = 0;
    public static double FEEDBACK_DERIVATIVE_GAIN = 0.0003;
    public static double FEED_FORWARD_GAIN = 0.1;
    public static double VELOCITY_GAIN = 0.0001;
    public static double ACCELERATION_GAIN = 0.00001;


    public static double ACCELERATION_HANG = 600;
    public static double DECELERATION_HANG = 500;
    public static double MAX_VELOCITY_HANG = 120;
    public static double FEEDBACK_PROPORTIONAL_GAIN_HANG = 0.021;
    public static double FEEDBACK_DERIVATIVE_GAIN_HANG = 0.0003;
    public static double FEEDBACK_INTEGRAL_GAIN_HANG = 0.000001;
    public static double FEED_FORWARD_GAIN_HANG = -0.7;
    public static double VELOCITY_GAIN_HANG = 0.00017;
    public static double ACCELERATION_GAIN_HANG = 0.00001;


    public static double ACCELERATION_HANG_FAST = 2600;
    public static double DECELERATION_HANG_FAST = 2200;
    public static double MAX_VELOCITY_HANG_FAST = 1200;
    public static double FEEDBACK_PROPORTIONAL_GAIN_HANG_FAST = 0.035;
    public static double FEEDBACK_INTEGRAL_GAIN_HANG_FAST = 0.00001;
    public static double VELOCITY_GAIN_HANG_FAST = 0.00017;





    public static double CREEP = 60;

    public static double ERROR_MARGIN = 20;
    public static double ERROR_TIMEOUT_MILLIS = 1000; // ms before position is automatically classified as reached

    public static double MIN_POSITION = 0;
    public static double MIN_MANUAL_ADJUST_POSITION = 160;
    public static double HORIZONTAL_EXTENSION_LIMIT = 650;
    public static double MAX_POSITION = 1230;
    public static double TICKS_PER_IN = 17.97 * 2.54;
    public static double MAX_EXTENSION_IN = HORIZONTAL_EXTENSION_LIMIT / TICKS_PER_IN;

    public static double INTAKE_SPECIMEN = 0.05;


    public enum TargetPosition {
        RETRACTED(0),
        INTAKE_SAMPLE(0.45), // 30.8cm
        SCORE_BUCKET_HIGH(1.02),
        PREPARE_SPECIMEN_HIGH(0.45),
        SCORE_SPECIMEN_HIGH(0.165),
        INTAKE_SPECIMEN(0.45);

        public final double extension;

        TargetPosition(double extension) {
            this.extension = extension;
        }
    }


    public enum OperationMode {
        NORMAL,
        HANG
    }
}
