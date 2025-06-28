package org.firstinspires.ftc.teamcode.subsystems.arm.slide;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;
import static org.firstinspires.ftc.teamcode.helpers.utils.GlobalConfig.DEBUG_MODE;
import static org.firstinspires.ftc.teamcode.subsystems.arm.MainArmSubsystem.mapToRange;
import static org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideConfiguration.ACCELERATION;
import static org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideConfiguration.ACCELERATION_GAIN;
import static org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideConfiguration.ACCELERATION_GAIN_HANG;
import static org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideConfiguration.ACCELERATION_HANG;
import static org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideConfiguration.CREEP;
import static org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideConfiguration.DECELERATION;
import static org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideConfiguration.DECELERATION_HANG;
import static org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideConfiguration.ENCODER_NAME;
import static org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideConfiguration.ERROR_MARGIN;
import static org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideConfiguration.FEEDBACK_DERIVATIVE_GAIN;
import static org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideConfiguration.FEEDBACK_DERIVATIVE_GAIN_HANG;
import static org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideConfiguration.FEEDBACK_DERIVATIVE_GAIN_HOLD_POINT;
import static org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideConfiguration.FEEDBACK_INTEGRAL_GAIN;
import static org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideConfiguration.FEEDBACK_INTEGRAL_GAIN_HANG;
import static org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideConfiguration.FEEDBACK_INTEGRAL_GAIN_HOLD_POINT;
import static org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideConfiguration.FEEDBACK_PROPORTIONAL_GAIN;
import static org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideConfiguration.FEEDBACK_PROPORTIONAL_GAIN_HANG;
import static org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideConfiguration.FEEDBACK_PROPORTIONAL_GAIN_HOLD_POINT;
import static org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideConfiguration.FEED_FORWARD_GAIN;
import static org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideConfiguration.FEED_FORWARD_GAIN_HANG;
import static org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideConfiguration.LIMIT_SW_NAME;
import static org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideConfiguration.MAX_POSITION;
import static org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideConfiguration.MAX_VELOCITY;
import static org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideConfiguration.MAX_VELOCITY_HANG;
import static org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideConfiguration.MIN_POSITION;
import static org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideConfiguration.MOTOR_NAME_0;
import static org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideConfiguration.MOTOR_NAME_1;
import static org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideConfiguration.MOTOR_NAME_2;
import static org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideConfiguration.VELOCITY_GAIN;
import static org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideConfiguration.VELOCITY_GAIN_HANG;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.helpers.utils.MotionProfile;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmState;
import org.firstinspires.ftc.teamcode.subsystems.arm.MainArmConfiguration.OPERATION_MODE;
import org.firstinspires.ftc.teamcode.subsystems.arm.MainArmSubsystem;

import java.util.logging.Level;

@Config
public class ArmSlideSubsystem {
    private DcMotorEx extensionMotor0, extensionMotor1, extensionMotor2;
    private DcMotorEx extensionEncoder;

    private TouchSensor limitSwitch;

    private MotionProfile motionProfile;
    private static volatile double encoderPosition;
    private static volatile double encoderOffset;

    private boolean overridePowerState = false;
    private double overridePowerValue = 0;

    private double powerLimit = 1;

    private double feedForwardGain = FEED_FORWARD_GAIN;
    private boolean limitSwitchPressed = true;
    private boolean prevLimitSwitchPressed = false;
    private boolean encoderReset = true;

    private OPERATION_MODE operationMode = OPERATION_MODE.NORMAL;
    private OPERATION_MODE prevOperationMode = OPERATION_MODE.NORMAL;

    private ElapsedTime timer = new ElapsedTime();
    private boolean disableThirdMotor = false;

    private double Hangpower = 0.3;
    private double delta = 0.055;
    private double direction = 1;
    private double maxPower = 0;
    VoltageSensor voltageSensor;

    private PIDController holdPointPID = new PIDController(FEEDBACK_PROPORTIONAL_GAIN_HOLD_POINT, FEEDBACK_INTEGRAL_GAIN_HOLD_POINT, FEEDBACK_DERIVATIVE_GAIN_HOLD_POINT);


    public ArmSlideSubsystem(HardwareMap hardwareMap) {
        extensionMotor0 = hardwareMap.get(DcMotorEx.class, MOTOR_NAME_0);
        extensionMotor1 = hardwareMap.get(DcMotorEx.class, MOTOR_NAME_1);
        extensionMotor2 = hardwareMap.get(DcMotorEx.class, MOTOR_NAME_2);

        limitSwitch = hardwareMap.get(TouchSensor.class, LIMIT_SW_NAME);

        extensionMotor0.setDirection(DcMotorEx.Direction.FORWARD);
        extensionMotor1.setDirection(DcMotorEx.Direction.FORWARD);
        extensionMotor2.setDirection(DcMotorEx.Direction.FORWARD);

        extensionMotor0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extensionMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extensionMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        extensionEncoder = hardwareMap.get(DcMotorEx.class, ENCODER_NAME);
        this.voltageSensor = (VoltageSensor) hardwareMap.voltageSensor.iterator().next();

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


        motionProfile.enableTelemetry(DEBUG_MODE);
        timer.reset();

        if  (ArmState.isCurrentState(ArmState.State.SAMPLE_SCORE, ArmState.State.SPECIMEN_SCORE_BACK)){
            extensionEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motionProfile.setTargetPosition(getExtension());
        }

        else {
            encoderOffset = 0;
            extensionEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            extensionEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motionProfile.setTargetPosition(0);
        }

        updateEncoderPosition();
    }


    public void setTargetPosition(double position) {
        VLRSubsystem.getLogger(MainArmSubsystem.class).log(Level.WARNING, "NEW SLIDE EXTENSION OF " + position + " JUST SET");

        position = mapToRange(position, 0, 1, MIN_POSITION, MAX_POSITION);
        updateCoefficientsForOperationMode();
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


    public void enablePowerOverride(double power){
        overridePowerState = true;
        overridePowerValue = power;
    }

    public void disablePowerOverride(){
        overridePowerState = false;
        overridePowerValue = 0;
    }

    public boolean getPowerOverride(){
        return overridePowerState;
    }

    private void setMotorPower(double power) {
        extensionMotor0.setPower(power);
        extensionMotor1.setPower(power);
        extensionMotor2.setPower(power);
    }

    public boolean getLimitSwitchState(){
        return limitSwitchPressed;
    }

    public double getT(){
        return motionProfile.getT();
    }

    private void resetEncoder(){
        encoderOffset = extensionEncoder.getCurrentPosition();
    }

    public void updateCoefficientsForOperationMode(){
        if (operationMode == OPERATION_MODE.HANG) {
            setHangCoefficients();
        }
        else if (operationMode == OPERATION_MODE.NORMAL || operationMode == OPERATION_MODE.NORMAL_SLOWER) {
            setDefaultCoefficients();
        }
    }


    private void updateEncoderPosition(){
        encoderPosition = (extensionEncoder.getCurrentPosition() - encoderOffset) / 8192d;
    }


    public void setPowerLimit(double powerLimit){
        this.powerLimit = powerLimit;
    }


    public void setThirdMotorEnable(boolean state){
        disableThirdMotor = state;
    }


    public double getCorrectedMaxPullPower(){
        //TANKS PERFORMANCE, THESE READS TAKE 3ms EACH
        double voltage = voltageSensor.getVoltage();
        double current = extensionMotor0.getCurrent(CurrentUnit.AMPS) + extensionMotor1.getCurrent(CurrentUnit.AMPS) + extensionMotor2.getCurrent(CurrentUnit.AMPS);
        double outputPower = voltage * current;

        // Compare to last measurement
        if (outputPower > maxPower) {maxPower = outputPower;}
        else{
            direction *= -1;
        }
        Hangpower += delta * direction;
        Hangpower = clamp(Hangpower, -1, 1);

        System.out.println("VOLTAGE: " + voltage + " CURRENT: " + current + " POWER: " + outputPower + " MOTOR POWER: " + Hangpower);
        return Hangpower;
    }


    public void periodic(double armAngleDegrees, OPERATION_MODE operationMode) {
        limitSwitchPressed = limitSwitch.isPressed();
        updateEncoderPosition();
        this.operationMode = operationMode;

//        if (operationMode == OPERATION_MODE.MAX_POWER_PULL && prevOperationMode != OPERATION_MODE.MAX_POWER_PULL){
//            bomboTimer.reset();
//            prevOperationMode = OPERATION_MODE.MAX_POWER_PULL;
//            //setMotorPower(-getCorrectedMaxPullPower());
//            //return;
//        }
//
//        if (operationMode == OPERATION_MODE.MAX_POWER_PULL){
//            setMotorPower(-bomboTimer.seconds() * 0.02);
//            double voltage = (double) controlHub.getBatteryVoltage() / 1000;
//            double current = extensionMotor0.getCurrent(CurrentUnit.AMPS) + extensionMotor1.getCurrent(CurrentUnit.AMPS) + extensionMotor2.getCurrent(CurrentUnit.AMPS);
//            double outputPower = voltage * current;
//
//            if (outputPower > maxPower) {
//                maxPower = outputPower;
//            }
//            System.out.println("VOLTAGE: " + voltage + " CURRENT: " + current + " POWER: " + outputPower + " MOTOR POWER: " + -bomboTimer.seconds() * 0.02);
//            return;
//        }
        if (operationMode == OPERATION_MODE.MAX_POWER_PULL){
            setMotorPower(-getCorrectedMaxPullPower());
            return;
        }

        if (operationMode == OPERATION_MODE.HOLD_POINT && prevOperationMode != OPERATION_MODE.HOLD_POINT){
            VLRSubsystem.getLogger(MainArmSubsystem.class).log(Level.WARNING, "SLIDES HOLDING POINT");
        }
        if (prevOperationMode != operationMode) {prevOperationMode = operationMode;}

        if (DEBUG_MODE){
            holdPointPID.setPID(FEEDBACK_PROPORTIONAL_GAIN_HOLD_POINT, FEEDBACK_INTEGRAL_GAIN_HOLD_POINT, FEEDBACK_DERIVATIVE_GAIN_HOLD_POINT);
        }

        if (operationMode == OPERATION_MODE.HANG){
            setHangCoefficients();
        }
        else{
            motionProfile.updateP(mapToRange(getExtension(), 0 ,1, FEEDBACK_PROPORTIONAL_GAIN * 1.2, FEEDBACK_PROPORTIONAL_GAIN));
        }

        double feedForwardPower = Math.sin(Math.toRadians(armAngleDegrees)) * feedForwardGain;
        double power = motionProfile.getPower(getPosition());

        if (getExtension() > 0.1){
            power += feedForwardPower;
        }

        if (overridePowerState) {
            setMotorPower(overridePowerValue);
            return;
        }

        if (operationMode == OPERATION_MODE.HANG){
            setMotorPower(power);
            System.out.println("SLIDE POWER " + power);
            return;
        }

        if (operationMode == OPERATION_MODE.HOLD_POINT && motionProfile.getTargetPosition() != 0) {
            power = holdPointPID.calculate(getPosition(), motionProfile.getTargetPosition()) + feedForwardPower;
        }
        power = clamp(power, -powerLimit, powerLimit);



        if (limitSwitchPressed && motionProfile.getTargetPosition() == 0) {
            if (!prevLimitSwitchPressed) {
                timer.reset();
            }

            else if (timer.seconds() < 0.5){
                setMotorPower(-0.2);
            }

            else if (timer.seconds() > 0.5) {
                setMotorPower(0);

                if (timer.seconds() > 1 && !encoderReset) {
                    resetEncoder();
                    encoderReset = true;
                }
            }
        }

        else {
            encoderReset = false;

            if (reachedTargetPosition()) {
                extensionMotor0.setPower(power);
                extensionMotor1.setPower(power);
                extensionMotor2.setPower(0);
            }
            else {
                extensionMotor1.setPower(power);
                extensionMotor2.setPower(power);
                if (disableThirdMotor){
                    extensionMotor0.setPower(0);
                }
                else{
                    extensionMotor0.setPower(power);
                }
            }
        }

        prevLimitSwitchPressed = limitSwitchPressed;

        if (DEBUG_MODE) {
            Telemetry telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());
            telemetry.addData("ARM SLIDE SUBSYSTEM POWER TIMER: ", timer.seconds());
            telemetry.addData("ARM SLIDE SUBSYSTEM ENCODER RESET: ", encoderReset ? 1 : 0);
            telemetry.addData("ARM SLIDE SUBSYSTEM LIMIT SWITCH STATE: ", limitSwitchPressed);
            telemetry.addData("ARM SLIDE SUBSYSTEM PREV LIMIT SWITCH STATE: ", prevLimitSwitchPressed);
        }
    }
}