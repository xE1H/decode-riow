package org.firstinspires.ftc.teamcode.subsystems.arm.rotator;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;
import static org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorConfiguration.*;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.helpers.utils.MotionProfile;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmState;
import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideSubsystem;


public class ArmRotatorSubsystem extends VLRSubsystem<ArmRotatorSubsystem> {
    private DcMotorEx motor;
    private DcMotorEx thoughBoreEncoder;

    private MotionProfile motionProfile;
    private ArmSlideSubsystem slideSubsystem;

    private double encoderPosition = 0;

    private boolean motorResetEnabled = false;
    private double feedForwardGain = FEEDFORWARD_GAIN;

    private boolean reachedPosition = true;
    private boolean prevReachedPosition = true;
    private ElapsedTime timer = new ElapsedTime();


    public static double mapToRange(double value, double minInput, double maxInput, double minOutput, double maxOutput) {
        if (minInput == maxInput) {
            throw new IllegalArgumentException("inMIN and inMax cant be the same");
        }
        return minOutput + ((value - minInput) * (maxOutput - minOutput)) / (maxInput - minInput);
    }


    protected void initialize(HardwareMap hardwareMap) {
        ArmState.resetAll();
        slideSubsystem = VLRSubsystem.getInstance(ArmSlideSubsystem.class);

        motor = hardwareMap.get(DcMotorEx.class, MOTOR_NAME);
        motor.setDirection(DcMotorEx.Direction.REVERSE);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        thoughBoreEncoder = hardwareMap.get(DcMotorEx.class, ENCODER_NAME);
        thoughBoreEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        thoughBoreEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motionProfile = new MotionProfile(
                FtcDashboard.getInstance().getTelemetry(),
                "ARM",
                MotionProfile.Type.ACCELERATION_LIMITED,
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
        timer.reset();
    }


    public void setTargetAngle(TargetAngle targetAngle) {
        motionProfile.setTargetPosition(clamp(targetAngle.angleDegrees, MIN_ANGLE, MAX_ANGLE));
    }


    public void setTargetPosition(double angleDegrees) {
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
        feedForwardGain = FEEDFORWARD_GAIN_HANG;
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
        feedForwardGain = FEEDFORWARD_GAIN;
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


    public void setMappedCoefficients() {
        double slidePosition = slideSubsystem.getExtension();
        double p = mapToRange(slidePosition, 0, 1, FEEDBACK_PROPORTIONAL_GAIN, EXTENDED_FEEDBACK_PROPORTIONAL_GAIN);
        double i = mapToRange(slidePosition, 0, 1, FEEDBACK_INTEGRAL_GAIN, EXTENDED_FEEDBACK_INTEGRAL_GAIN);
        double d = mapToRange(slidePosition, 0, 1, FEEDBACK_DERIVATIVE_GAIN, EXTENDED_FEEDBACK_DERIVATIVE_GAIN);
        double v = mapToRange(slidePosition, 0, 1, VELOCITY_GAIN, EXTENDED_VELOCITY_GAIN);
        double a = mapToRange(slidePosition, 0, 1, ACCELERATION_GAIN, EXTENDED_ACCELERATION_GAIN);

        double acceleration = mapToRange(slidePosition, 0, 1, ACCELERATION_JERK, EXTENDED_ACCELERATION_JERK);
        double deceleration = mapToRange(slidePosition, 0, 1, DECELERATION_JERK, EXTENDED_DECELERATION_JERK);
        double maxVelocity = mapToRange(slidePosition, 0, 1, MAX_VELOCITY, EXTENDED_MAX_VELOCITY);
        double feedforward = mapToRange(slidePosition, 0, 1, FEEDFORWARD_GAIN, EXTENDED_FEEDFORWARD_GAIN);

        motionProfile.updateCoefficients(acceleration, deceleration, maxVelocity, p, i, d, v, a);
        feedForwardGain = feedforward;
    }

    public void disableMotor() {
        motorResetEnabled = true;
        motor.setPower(0);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void reenableMotor() {
        motorResetEnabled = false;

        thoughBoreEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        thoughBoreEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void periodic() {
        if (motorResetEnabled) return;
        encoderPosition = -thoughBoreEncoder.getCurrentPosition();

        double currentAngle = getAngleDegrees();

        double feedForwardPower = Math.cos(Math.toRadians(currentAngle)) * feedForwardGain;
        double power = motionProfile.getPower(currentAngle) + feedForwardPower;
        power = clamp(power, -1, 1);

        if (slideSubsystem.getOperationMode() == ArmSlideConfiguration.OperationMode.NORMAL) {
            setDefaultCoefficients();

            boolean reachedTarget = reachedTargetPosition();
            if (!reachedTarget || !prevReachedPosition){
                timer.reset();
            }
            prevReachedPosition = reachedTarget;

            if (reachedTarget && motionProfile.getTargetPosition() == TargetAngle.RETRACT.angleDegrees && timer.seconds() > 1){
                power = 0;
            }
        }

        motor.setPower(power);
        slideSubsystem.periodic(currentAngle);
    }
}