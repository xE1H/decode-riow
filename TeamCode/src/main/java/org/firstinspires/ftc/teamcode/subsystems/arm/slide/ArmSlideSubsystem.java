package org.firstinspires.ftc.teamcode.subsystems.arm.slide;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;
import static org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorSubsystem.mapToRange;
import static org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideConfiguration.*;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.teamcode.helpers.utils.MotionProfile;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmState;

@Config
public class ArmSlideSubsystem {
    private DcMotorSimple extensionMotor0, extensionMotor1, extensionMotor2;
    private DcMotorEx extensionEncoder;

    private TouchSensor limitSwitch;

    private MotionProfile motionProfile;
    private double encoderPosition = 0;

    private double encoderOffset = 0;
    private boolean overridePower = false;
    private boolean holdingPosition = false;
    private double feedForwardGain = FEED_FORWARD_GAIN;



    public ArmSlideSubsystem(HardwareMap hardwareMap) {
        ArmState.resetAll();

        extensionMotor0 = hardwareMap.get(DcMotorSimple.class, MOTOR_NAME_0);
        extensionMotor1 = hardwareMap.get(DcMotorSimple.class, MOTOR_NAME_1);
        extensionMotor2 = hardwareMap.get(DcMotorSimple.class, MOTOR_NAME_2);

        limitSwitch = hardwareMap.get(TouchSensor.class, LIMIT_SW_NAME);

        extensionMotor0.setDirection(DcMotorSimple.Direction.FORWARD);
        extensionMotor1.setDirection(DcMotorSimple.Direction.FORWARD);
        extensionMotor2.setDirection(DcMotorSimple.Direction.FORWARD);

        extensionEncoder = hardwareMap.get(DcMotorEx.class, ENCODER_NAME);
        extensionEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        extensionEncoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        motionProfile = new MotionProfile(
                FtcDashboard.getInstance().getTelemetry(),
                "SLIDE",
                MotionProfile.Type.ACCELERATION_LIMITED,
                ACCELERATION,
                DECELERATION,
                MAX_VELOCITY,
                CREEP,
                FEEDBACK_PROPORTIONAL_GAIN,
                FEEDBACK_INTEGRAL_GAIN,
                FEEDBACK_DERIVATIVE_GAIN,
                VELOCITY_GAIN,
                ACCELERATION_GAIN);

        motionProfile.enableTelemetry(true);
    }



    public void setTargetPosition(double position) {
        holdingPosition = false;
        setDefaultCoefficients();

        position = mapToRange(position, 0, 1, MIN_POSITION, MAX_POSITION);
        motionProfile.setTargetPosition(clamp(position, MIN_POSITION, MAX_POSITION));
    }


    public boolean reachedTargetPosition() {
        return reachedPosition(getTargetPosition());
    }

    public boolean reachedPosition(double position) {
        return Math.abs(getPosition() - position) <= ERROR_MARGIN;
    }


    public double getPosition() {
        return encoderPosition;
    }


    public double getExtension(){
        return mapToRange(getPosition(), MIN_POSITION, MAX_POSITION, 0, 1);
    }


    public double getTargetPosition() {
        return motionProfile.getTargetPosition();
    }


    public double getTargetExtension() {
        return mapToRange(getPosition(), MIN_POSITION, MAX_POSITION, 0, 1);
    }


    public void setHangCoefficients() {
        feedForwardGain = FEED_FORWARD_GAIN_HANG;
        motionProfile.updateCoefficients(
                ACCELERATION_HANG,
                DECELERATION_HANG,
                MAX_VELOCITY_HANG,
                FEEDBACK_PROPORTIONAL_GAIN_HANG,
                FEEDBACK_INTEGRAL_GAIN_HANG,
                FEEDBACK_DERIVATIVE_GAIN_HANG,
                VELOCITY_GAIN_HANG,
                ACCELERATION_GAIN_HANG);
    }


    public void setDefaultCoefficients() {
        feedForwardGain = FEED_FORWARD_GAIN;
        motionProfile.updateCoefficients(
                ACCELERATION,
                DECELERATION,
                MAX_VELOCITY,
                FEEDBACK_PROPORTIONAL_GAIN,
                FEEDBACK_INTEGRAL_GAIN,
                FEEDBACK_DERIVATIVE_GAIN,
                VELOCITY_GAIN,
                ACCELERATION_GAIN);
    }



    public void setPowerOverride(boolean condition){
        overridePower = condition;
    }


    public boolean getPowerOverride(){
        return overridePower;
    }


    public void setMotorPower(double power) {
        extensionMotor0.setPower(power);
        extensionMotor1.setPower(power);
        extensionMotor2.setPower(power);
    }


    public void checkLimitSwitch() {
        if (limitSwitch.isPressed()) {encoderOffset = extensionEncoder.getCurrentPosition();}
    }

    public boolean getLimitSwitchState(){
        return limitSwitch.isPressed();
    }

    public double getT(){
        return motionProfile.getT();
    }

    private void updateCoefficients(){
        if (motionProfile.getT() == 1 && !holdingPosition){
            motionProfile.updateCoefficients(0, 0, 0, FEEDBACK_PROPORTIONAL_GAIN_HOLD_POINT, FEEDBACK_INTEGRAL_GAIN_HOLD_POINT, FEEDBACK_DERIVATIVE_GAIN_HOLD_POINT, 0, 0);
            holdingPosition = true;
        }
    }


    public void periodic(double armAngleDegrees) {
        checkLimitSwitch();
        encoderPosition = -(extensionEncoder.getCurrentPosition() - encoderOffset) / 8192d;

        double feedForwardPower = Math.sin(Math.toRadians(armAngleDegrees)) * feedForwardGain;
        double power = motionProfile.getPower(getPosition()) + feedForwardPower;
        power = clamp(power, -1, 1);

        //setMotorPower(power);

//        if (!overridePower) {
//            if (operationMode == OperationMode.NORMAL) {
//                setDefaultCoefficients();
//
//                if (reachedTargetPositionNoOverride()) {
//                    extensionMotor0.setPower(0);
//
//                    if (getTargetExtension() == TargetPosition.RETRACTED.extension) {
//                        extensionMotor1.setPower(0);
//                        extensionMotor2.setPower(0);
//
//                    } else{
//                        extensionMotor1.setPower(power);
//                        extensionMotor2.setPower(power);
//                    }
//
//                } else setMotorPower(power);
//            } else{
//                setMotorPower(power);
//            }
//        }

        //updateCoefficients();
    }
}