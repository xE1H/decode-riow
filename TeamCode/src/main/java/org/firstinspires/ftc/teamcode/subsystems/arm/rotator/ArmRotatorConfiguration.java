package org.firstinspires.ftc.teamcode.subsystems.arm.rotator;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ArmRotatorConfiguration {

    public static String MOTOR_NAME = "MotorRotator";
    public static String ENCODER_NAME = "MotorRightBack"; // PORT 3 CONTOR LHUB

    public static double ACCELERATION = 8000;
    public static double DECELERATION = 1300;
    public static double MAX_VELOCITY = 310;

    public static double FEEDBACK_PROPORTIONAL_GAIN = 0.095;
    public static double FEEDBACK_INTEGRAL_GAIN = 0;
    public static double FEEDBACK_DERIVATIVE_GAIN = 0.00045;
    public static double VELOCITY_GAIN = 0.003;
    public static double ACCELERATION_GAIN = 0.00005;

    public static double RETRACTED_FEEDFORWARD_GAIN = 0.12;
    public static double EXTENDED_FEEDFORWARD_GAIN = 0.3;

    public static double FEEDBACK_PROPORTIONAL_GAIN_HANG = 0.4;
    public static double FEEDBACK_INTEGRAL_GAIN_HANG = 0.007;

    public static double ERROR_MARGIN = 20;

    public static double MIN_ANGLE = 0;
    public static double MAX_ANGLE = 150;

    public static double ENCODER_TICKS_PER_ROTATION = 8192;


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
