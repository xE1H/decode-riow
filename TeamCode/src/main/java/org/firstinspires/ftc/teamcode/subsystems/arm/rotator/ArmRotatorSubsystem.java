package org.firstinspires.ftc.teamcode.subsystems.arm.rotator;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;
import static org.firstinspires.ftc.teamcode.helpers.utils.MotionProfile.FeedforwardType.COSINE;
import static org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorConfiguration.*;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.helpers.utils.GlobalConfig;
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


    public static double mapToRange(double value, double minInput, double maxInput, double minOutput, double maxOutput) {
        if (minInput == maxInput) {
            throw new IllegalArgumentException("inMIN and inMax cant be the same");
        }
        return minOutput + ((value - minInput) * (maxOutput - minOutput)) / (maxInput - minInput);
    }


    protected void initialize(HardwareMap hardwareMap) {
        ArmState.resetAll();
        Telemetry telemetry = FtcDashboard.getInstance().getTelemetry();
        slideSubsystem = VLRSubsystem.getInstance(ArmSlideSubsystem.class);

        motor = hardwareMap.get(DcMotorEx.class, MOTOR_NAME);
        motor.setDirection(DcMotorEx.Direction.REVERSE);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        thoughBoreEncoder = hardwareMap.get(DcMotorEx.class, ENCODER_NAME);
        thoughBoreEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        thoughBoreEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motionProfile = new MotionProfile(telemetry, "ARM", ACCELERATION, DECELERATION, MAX_VELOCITY, FEEDBACK_PROPORTIONAL_GAIN, FEEDBACK_INTEGRAL_GAIN, FEEDBACK_DERIVATIVE_GAIN, VELOCITY_GAIN, ACCELERATION_GAIN, COSINE);
        motionProfile.enableTelemetry(true);
    }


    public void setTargetAngle(TargetAngle targetAngle) {
        motionProfile.setCurrentTargetPosition(clamp(targetAngle.angleDegrees, MIN_ANGLE, MAX_ANGLE));
    }


    public void setTargetPosition(ArmSlideConfiguration.TargetPosition targetPosition) {
        slideSubsystem.setTargetPosition(targetPosition);
    }


    public void setTargetPosition(double angleDegrees) {
        motionProfile.setCurrentTargetPosition(clamp(angleDegrees, MIN_ANGLE, MAX_ANGLE));

    }


    public double getAngleDegrees() {
        if(GlobalConfig.INVERTED_ENCODERS){
            return -encoderPosition / ENCODER_TICKS_PER_ROTATION * 360d;
        }

        return encoderPosition / ENCODER_TICKS_PER_ROTATION * 360d;
    }

    public boolean reachedTargetPosition() {
        return reachedPosition(motionProfile.getTargetPosition());
    }


    public boolean reachedPosition(double angleDegrees) {
        return Math.abs(getAngleDegrees() - angleDegrees) < ERROR_MARGIN;
    }

    public void setHangCoefficients(){
        motionProfile.updateCoefficients(ACCELERATION_HANG, DECELERATION_HANG, MAX_VELOCITY_HANG, FEEDBACK_PROPORTIONAL_GAIN_HANG, FEEDBACK_INTEGRAL_GAIN_HANG, FEEDBACK_DERIVATIVE_GAIN, VELOCITY_GAIN, ACCELERATION_GAIN);
        motionProfile.setFeedForwardGain(FEEDFORWARD_GAIN_HANG);
    }

    public void setDefaultCoefficients(){
        double slidePosition = slideSubsystem.getPosition();
        double p = mapToRange(slidePosition, ArmSlideConfiguration.MIN_POSITION, ArmSlideConfiguration.MAX_POSITION, FEEDBACK_PROPORTIONAL_GAIN, EXTENDED_FEEDBACK_PROPORTIONAL_GAIN);
        double i = mapToRange(slidePosition, ArmSlideConfiguration.MIN_POSITION, ArmSlideConfiguration.MAX_POSITION, FEEDBACK_INTEGRAL_GAIN, EXTENDED_FEEDBACK_INTEGRAL_GAIN);
        double d = mapToRange(slidePosition, ArmSlideConfiguration.MIN_POSITION, ArmSlideConfiguration.MAX_POSITION, FEEDBACK_DERIVATIVE_GAIN, EXTENDED_FEEDBACK_DERIVATIVE_GAIN);
        double v = mapToRange(slidePosition, ArmSlideConfiguration.MIN_POSITION, ArmSlideConfiguration.MAX_POSITION, VELOCITY_GAIN, EXTENDED_VELOCITY_GAIN);
        double a = mapToRange(slidePosition, ArmSlideConfiguration.MIN_POSITION, ArmSlideConfiguration.MAX_POSITION, ACCELERATION_GAIN, EXTENDED_ACCELERATION_GAIN);

        double acceleration = mapToRange(slidePosition, ArmSlideConfiguration.MIN_POSITION, ArmSlideConfiguration.MAX_POSITION, ACCELERATION, EXTENDED_ACCELERATION);
        double deceleration = mapToRange(slidePosition, ArmSlideConfiguration.MIN_POSITION, ArmSlideConfiguration.MAX_POSITION, DECELERATION, EXTENDED_DECELERATION);
        double maxVelocity = mapToRange(slidePosition, ArmSlideConfiguration.MIN_POSITION, ArmSlideConfiguration.MAX_POSITION, MAX_VELOCITY, EXTENDED_MAX_VELOCITY);
        double feedforward = mapToRange(slidePosition, ArmSlideConfiguration.MIN_POSITION, ArmSlideConfiguration.MAX_POSITION, FEEDFORWARD_GAIN, EXTENDED_FEEDFORWARD_GAIN);

        motionProfile.updateCoefficients(acceleration, deceleration, maxVelocity, p, i, d, v, a);
        motionProfile.setFeedForwardGain(feedforward);
    }


    @Override
    public void periodic() {
        encoderPosition = thoughBoreEncoder.getCurrentPosition();

        double currentAngle = getAngleDegrees();

        FtcDashboard.getInstance().getTelemetry().addData("Rotator Angle", currentAngle);
        double power = motionProfile.getPower(currentAngle);

        if (slideSubsystem.getOperationMode() == ArmSlideConfiguration.OperationMode.NORMAL) {
            setDefaultCoefficients();

            if (motionProfile.getTargetPosition() == TargetAngle.DOWN.angleDegrees && reachedTargetPosition()) {
                power = 0;
            }
        }
        else{
            setHangCoefficients();
        }
        motor.setPower(power);
        slideSubsystem.periodic(currentAngle);
    }
}
