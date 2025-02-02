package org.firstinspires.ftc.teamcode.subsystems.arm.rotator;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ArmRotatorConfiguration {

    public static String MOTOR_NAME = "MotorRotator";
    public static String ENCODER_NAME = "MotorLeftBack";

    //ACCELERATION PROFILE CONSTANTS:
//    public static double ACCELERATION = 8000;
//    public static double DECELERATION = 1100;
//    public static double MAX_VELOCITY = 300;
//
//    public static double FEEDBACK_PROPORTIONAL_GAIN = 0.075;
//    public static double FEEDBACK_INTEGRAL_GAIN = 0;
//    public static double FEEDBACK_DERIVATIVE_GAIN = 0.00045;
//    public static double VELOCITY_GAIN = 0.003;
//    public static double ACCELERATION_GAIN = 0.00005;
//    public static double FEEDFORWARD_GAIN = 0.12;


    //JERK PROFILE CONSTANTS:
    public static double ACCELERATION_JERK = 25000;
    public static double DECELERATION_JERK = 20000;
    public static double MAX_VELOCITY = 370;

    public static double FEEDBACK_PROPORTIONAL_GAIN = 0.07;
    public static double FEEDBACK_INTEGRAL_GAIN = 0;
    public static double FEEDBACK_DERIVATIVE_GAIN = 0.002;
    public static double VELOCITY_GAIN = 0.002;
    public static double ACCELERATION_GAIN = 0.00008;
    public static double FEEDFORWARD_GAIN = 0.13;


    public static double EXTENDED_ACCELERATION_JERK = 2000;
    public static double EXTENDED_DECELERATION_JERK = 900;
    public static double EXTENDED_MAX_VELOCITY = 70;
    public static double EXTENDED_FEEDBACK_PROPORTIONAL_GAIN = 0.068;
    public static double EXTENDED_FEEDBACK_INTEGRAL_GAIN = 0.00001;
    public static double EXTENDED_FEEDBACK_DERIVATIVE_GAIN = 0.0038;
    public static double EXTENDED_VELOCITY_GAIN = 0.00045;
    public static double EXTENDED_ACCELERATION_GAIN = 0.00005;
    public static double EXTENDED_FEEDFORWARD_GAIN = 0.3;



    public static double ACCELERATION_HANG = 1000;
    public static double DECELERATION_HANG = 500;
    public static double MAX_VELOCITY_HANG = 50;
    public static double FEEDBACK_PROPORTIONAL_GAIN_HANG = 0.3;
    public static double FEEDBACK_INTEGRAL_GAIN_HANG = 0;//.0001;
    public static double FEEDFORWARD_GAIN_HANG = -0;//.1;



    public static double ERROR_MARGIN = 3;

    public static double MIN_ANGLE = 0;
    public static double MAX_ANGLE = 150;

    public static double ENCODER_TICKS_PER_ROTATION = 8192;

    public static long ERROR_TIMEOUT_MILLIS = 2500;

    public static double PREPARE_SPECIMEN_HIGH = 103.5;

    public enum TargetAngle {
        RETRACT(0),
        INTAKE_SAMPLE(0),
        SCORE_SAMPLE_HIGH(110),
        PREPARE_SPECIMEN_HIGH(103.5),
        INTAKE_SPECIMEN(25);
      
        public final double angleDegrees;

        TargetAngle(double angleDegrees) {
            this.angleDegrees = angleDegrees;
        }
    }
}
