package org.firstinspires.ftc.teamcode.subsystems.arm.rotator;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;
import static org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorConfiguration.*;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.helpers.utils.MotionProfile;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmState;
import org.firstinspires.ftc.teamcode.subsystems.arm.MainArmSubsystem;

import java.util.logging.Level;


public class ArmRotatorSubsystem {
    private DcMotorSimple motor;
    private DcMotorEx thoughBoreEncoder;

    private MotionProfile motionProfile;

    private double encoderPosition = 0;
    private double encoderOffset = 0;

    private boolean holdingPosition = false;

    private ElapsedTime timer = new ElapsedTime();


    public static double mapToRange(double value, double minInput, double maxInput, double minOutput, double maxOutput) {
        if (minInput == maxInput) {
            throw new IllegalArgumentException("inMIN and inMax cant be the same");
        }
        return minOutput + ((value - minInput) * (maxOutput - minOutput)) / (maxInput - minInput);
    }


    public ArmRotatorSubsystem (HardwareMap hardwareMap) {
        ArmState.resetAll();

        motor = hardwareMap.get(DcMotorSimple.class, MOTOR_NAME);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);

        thoughBoreEncoder = hardwareMap.get(DcMotorEx.class, ENCODER_NAME);
        thoughBoreEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        thoughBoreEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motionProfile = new MotionProfile(
                FtcDashboard.getInstance().getTelemetry(),
                "ARM",
                MotionProfile.Type.JERK_LIMITED,
                ACCELERATION_JERK,
                DECELERATION_JERK,
                MAX_VELOCITY,
                ERROR_MARGIN,
                FEEDBACK_PROPORTIONAL_GAIN,
                FEEDBACK_INTEGRAL_GAIN,
                FEEDBACK_DERIVATIVE_GAIN,
                VELOCITY_GAIN,
                ACCELERATION_GAIN);

        motionProfile.enableTelemetry(true);
        motionProfile.setTargetPosition(0);
        timer.reset();
    }


    public void setTargetPosition(double angleDegrees, double slideExtension) {
        VLRSubsystem.getLogger(MainArmSubsystem.class).log(Level.WARNING, "NEW ROTATOR ANGLE OF " + angleDegrees + " JUST SET");

//        holdingPosition = false;
//
//        double p = mapToRange(slideExtension, 0, 1, FEEDBACK_PROPORTIONAL_GAIN, EXTENDED_FEEDBACK_PROPORTIONAL_GAIN);
//        double i = mapToRange(slideExtension, 0, 1, FEEDBACK_INTEGRAL_GAIN, EXTENDED_FEEDBACK_INTEGRAL_GAIN);
//        double d = mapToRange(slideExtension, 0, 1, FEEDBACK_DERIVATIVE_GAIN, EXTENDED_FEEDBACK_DERIVATIVE_GAIN);
//        double v = mapToRange(slideExtension, 0, 1, VELOCITY_GAIN, EXTENDED_VELOCITY_GAIN);
//        double a = mapToRange(slideExtension, 0, 1, ACCELERATION_GAIN, EXTENDED_ACCELERATION_GAIN);
//        double acceleration = mapToRange(slideExtension, 0, 1, ACCELERATION_JERK, EXTENDED_ACCELERATION_JERK);
//        double deceleration = mapToRange(slideExtension, 0, 1, DECELERATION_JERK, EXTENDED_DECELERATION_JERK);
//        double maxVelocity = mapToRange(slideExtension, 0, 1, MAX_VELOCITY, EXTENDED_MAX_VELOCITY);
//
//        motionProfile.updateCoefficients(acceleration, deceleration, maxVelocity, p, i, d, v, a);
        motionProfile.setTargetPosition(clamp(angleDegrees, MIN_ANGLE, MAX_ANGLE));
    }


    public double getAngleDegrees() {
        return encoderPosition / ENCODER_TICKS_PER_ROTATION * 360d;
    }

    public boolean reachedTargetPosition() {
        return reachedPosition(motionProfile.getTargetPosition());
    }


    public boolean reachedPosition(double angleDegrees) {
        return Math.abs(getAngleDegrees() - angleDegrees) < ERROR_MARGIN;
    }

    public void setHangCoefficients() {
        //feedForwardGain = FEEDFORWARD_GAIN_HANG;
        motionProfile.updateCoefficients(
                ACCELERATION_HANG,
                DECELERATION_HANG,
                MAX_VELOCITY_HANG,
                FEEDBACK_PROPORTIONAL_GAIN_HANG,
                FEEDBACK_INTEGRAL_GAIN_HANG,
                FEEDBACK_DERIVATIVE_GAIN,
                VELOCITY_GAIN,
                ACCELERATION_GAIN);
    }


    public void setDefaultCoefficients() {
        //feedForwardGain = FEEDFORWARD_GAIN;
        motionProfile.updateCoefficients(
                ACCELERATION_JERK,
                DECELERATION_JERK,
                MAX_VELOCITY,
                FEEDBACK_PROPORTIONAL_GAIN,
                FEEDBACK_INTEGRAL_GAIN,
                FEEDBACK_DERIVATIVE_GAIN,
                VELOCITY_GAIN,
                ACCELERATION_GAIN);
    }


    public void updateCoefficients(){
        if (motionProfile.getT() == 1 && !holdingPosition) {
            motionProfile.updateCoefficients(0, 0, 0, FEEDBACK_PROPORTIONAL_GAIN_HOLD_POINT, FEEDBACK_INTEGRAL_GAIN_HOLD_POINT, FEEDBACK_DERIVATIVE_GAIN_HOLD_POINT, 0, 0);
            holdingPosition = true;
        }
    }


    public double getT(){
        return motionProfile.getT();
    }

    public void periodic(double slideExtension) {
        encoderPosition = thoughBoreEncoder.getCurrentPosition() - encoderOffset;
        double currentAngle = getAngleDegrees();

        double feedForward = mapToRange(slideExtension, 0, 1, FEEDFORWARD_GAIN, EXTENDED_FEEDFORWARD_GAIN);

        double feedForwardPower = Math.cos(Math.toRadians(currentAngle)) * feedForward;
        double power = motionProfile.getPower(currentAngle) + feedForwardPower;
        power = clamp(power, -1, 1);

        //updateCoefficients();
//        if (slideSubsystem.getOperationMode() == ArmSlideConfiguration.OperationMode.NORMAL) {
//            setDefaultCoefficients();
//
//            boolean reachedTarget = reachedTargetPosition();
//            if (!reachedTarget || !prevReachedPosition){
//                timer.reset();
//            }
//            prevReachedPosition = reachedTarget;
//
//            if (reachedTarget && motionProfile.getTargetPosition() == TargetAngle.RETRACT.angleDegrees && timer.seconds() > 0.5){
//                power = 0;
//            }
//        }

        //motor.setPower(power);
    }
}