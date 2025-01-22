package org.firstinspires.ftc.teamcode.subsystems.arm.rotator;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ArmRotatorConfiguration {

    public static String MOTOR_NAME = "MotorRotator";
    public static String ENCODER_NAME = "MotorRightBack"; // PORT 3 CONTOR LHUB

    public static double ACCELERATION = 8000;
    public static double DECELERATION = 1300;
    public static double MAX_VELOCITY = 310;

    public static double FEEDBACK_PROPORTIONAL_GAIN = 0.075;
    public static double FEEDBACK_INTEGRAL_GAIN = 0.00001;
    public static double FEEDBACK_DERIVATIVE_GAIN = 0.00045;
    public static double VELOCITY_GAIN = 0.003;
    public static double ACCELERATION_GAIN = 0.00005;
    public static double FEEDFORWARD_GAIN = 0.12;


    public static double EXTENDED_ACCELERATION = 2000;
    public static double EXTENDED_DECELERATION = 200;
    public static double EXTENDED_MAX_VELOCITY = 60;
    public static double EXTENDED_FEEDBACK_PROPORTIONAL_GAIN = 0.06;
    public static double EXTENDED_FEEDBACK_INTEGRAL_GAIN = 0.000001;
    public static double EXTENDED_FEEDBACK_DERIVATIVE_GAIN = 0.0003;
    public static double EXTENDED_VELOCITY_GAIN = 0.002;
    public static double EXTENDED_ACCELERATION_GAIN = 0.00003;
    public static double EXTENDED_FEEDFORWARD_GAIN = 0.3;



    public static double ACCELERATION_HANG = 1000;
    public static double DECELERATION_HANG = 500;
    public static double MAX_VELOCITY_HANG = 50;
    public static double FEEDBACK_PROPORTIONAL_GAIN_HANG = 0.4;
    public static double FEEDBACK_INTEGRAL_GAIN_HANG = 0.0001;
    public static double FEEDFORWARD_GAIN_HANG = -0.1;



    public static double ERROR_MARGIN = 5;

    public static double MIN_ANGLE = 0;
    public static double MAX_ANGLE = 150;

    public static double ENCODER_TICKS_PER_ROTATION = 8192;

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
