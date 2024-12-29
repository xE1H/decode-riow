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
import org.firstinspires.ftc.teamcode.helpers.utils.MotionProfile;
import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideSubsystem;

public class ArmRotatorSubsystem extends VLRSubsystem<ArmRotatorSubsystem> {
    private DcMotorEx motor;
    private DcMotorEx thoughBoreEncoder;

    private MotionProfile motionProfile;
    private ArmSlideSubsystem slideSubsystem;

    private RotatorState rotatorState;
    private ArmState armState = ArmState.IN_ROBOT;

    private double encoderPosition = 0;

    public static double mapToRange(double value, double minInput, double maxInput, double minOutput, double maxOutput) {
        if (minInput == maxInput) {
            throw new IllegalArgumentException("inMIN and inMax cant be the same");
        }
        return minOutput + ((value - minInput) * (maxOutput - minOutput)) / (maxInput - minInput);
    }

    protected void initialize(HardwareMap hardwareMap) {
        Telemetry telemetry = FtcDashboard.getInstance().getTelemetry();
        slideSubsystem = VLRSubsystem.getInstance(ArmSlideSubsystem.class);

        motor = hardwareMap.get(DcMotorEx.class, MOTOR_NAME);
        motor.setDirection(DcMotorEx.Direction.REVERSE);

        thoughBoreEncoder = hardwareMap.get(DcMotorEx.class, ENCODER_NAME);
        thoughBoreEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        thoughBoreEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motionProfile = new MotionProfile(telemetry, "ARM", ACCELERATION, DECELERATION, MAX_VELOCITY, FEEDBACK_PROPORTIONAL_GAIN, FEEDBACK_INTEGRAL_GAIN, FEEDBACK_DERIVATIVE_GAIN, VELOCITY_GAIN, ACCELERATION_GAIN, COSINE);
        motionProfile.enableTelemetry(true);

        rotatorState = RotatorState.IN_ROBOT;
    }

    public void setTargetAngle(TargetAngle targetAngle) {
        motionProfile.setCurrentTargetPosition(clamp(targetAngle.angleDegrees, MIN_ANGLE, MAX_ANGLE));
    }

    public void setTargetPosition(ArmSlideConfiguration.TargetPosition targetPosition) {
        slideSubsystem.setTargetPosition(targetPosition);
    }

    public void setTargetPosition(double angleDegrees) {
        slideSubsystem.setTargetPosition(angleDegrees);
    }

    public double getAngleDegrees() {
        return -encoderPosition / ENCODER_TICKS_PER_ROTATION * 360d;
    }

    public boolean reachedTargetPosition() {
        return reachedPosition(motionProfile.getTargetPosition());
    }

    public boolean reachedPosition(double angleDegrees) {
        return Math.abs(getAngleDegrees() - angleDegrees) < ERROR_MARGIN;
    }

    @Override
    public void periodic() {
        encoderPosition = thoughBoreEncoder.getCurrentPosition();
        motionProfile.updateCoefficients(ACCELERATION, DECELERATION, MAX_VELOCITY, FEEDBACK_PROPORTIONAL_GAIN, FEEDBACK_INTEGRAL_GAIN, FEEDBACK_DERIVATIVE_GAIN, VELOCITY_GAIN, ACCELERATION_GAIN);

        double currentAngle = getAngleDegrees();
        double currentFeedForwardGain = mapToRange(slideSubsystem.getPosition(), ArmSlideConfiguration.MIN_POSITION, ArmSlideConfiguration.MAX_POSITION, RETRACTED_FEEDFORWARD_GAIN, EXTENDED_FEEDFORWARD_GAIN);

        motionProfile.setFeedForwardGain(currentFeedForwardGain);

        double power = motionProfile.getPower(currentAngle);

        if (motionProfile.getTargetPosition() == TargetAngle.DOWN.angleDegrees && reachedPosition(TargetAngle.DOWN.angleDegrees)){
            power = 0;
        }
        motor.setPower(power);
        slideSubsystem.periodic(currentAngle);
    }


    public RotatorState getRotatorState() {
        return this.rotatorState;
    }

    public void setRotatorState(RotatorState state) {
        this.rotatorState = state;
    }

    public ArmState getArmState(){
        return this.armState;
    }

    public void setArmState(ArmState armState){
        this.armState = armState;
    }
}
