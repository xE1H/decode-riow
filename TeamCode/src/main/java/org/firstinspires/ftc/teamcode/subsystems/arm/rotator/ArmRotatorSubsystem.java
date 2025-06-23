package org.firstinspires.ftc.teamcode.subsystems.arm.rotator;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;
import static org.firstinspires.ftc.teamcode.helpers.utils.GlobalConfig.DEBUG_MODE;
import static org.firstinspires.ftc.teamcode.subsystems.arm.MainArmSubsystem.mapToRange;
import static org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorConfiguration.*;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.helpers.utils.MotionProfile;
import org.firstinspires.ftc.teamcode.helpers.utils.PidController;
import org.firstinspires.ftc.teamcode.subsystems.arm.ArmState;
import org.firstinspires.ftc.teamcode.subsystems.arm.MainArmConfiguration.OPERATION_MODE;
import org.firstinspires.ftc.teamcode.subsystems.arm.MainArmSubsystem;

import java.util.logging.Level;


public class ArmRotatorSubsystem {
    private final DcMotorEx motor;
    private final DcMotorEx thoughBoreEncoder;

    private static volatile double encoderPosition;
    private static volatile double encoderOffset;

    private final PidController holdPointController = new PidController(HOLD_POINT_PROPORTIONAL, 0, HOLD_POINT_DERIVATIVE);

    private final ElapsedTime timer = new ElapsedTime();
    private boolean encoderReset = false;

    private final RevTouchSensor breamBreak;
    private boolean prevBreamBreakState = false;

    private OPERATION_MODE operationMode = OPERATION_MODE.NORMAL;
    private MotionProfile motionProfile;

    private boolean powerOverride_state = false;
    private double powerOverride_value = 0;
    private double powerLimit = 1;


    public ArmRotatorSubsystem (HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, MOTOR_NAME);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);

        thoughBoreEncoder = hardwareMap.get(DcMotorEx.class, ENCODER_NAME);
        breamBreak = hardwareMap.get(RevTouchSensor.class, BEAM_BREAK_NAME);

        motionProfile = new MotionProfile(
                FtcDashboard.getInstance().getTelemetry(),
                "ROTATOR",
                MotionProfile.Type.ACCELERATION_LIMITED,
                ACCELERATION,
                ACCELERATION,
                NOMINAL_VELOCITY,
                0,
                PROPORTIONAL,
                0,
                DERIVATIVE,
                0,
                0);

        motionProfile.enableTelemetry(DEBUG_MODE);
        timer.reset();

        if  (ArmState.isCurrentState(ArmState.State.SAMPLE_SCORE, ArmState.State.SPECIMEN_SCORE_BACK)){
            thoughBoreEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            updateEncoderPosition();
            setTargetPosition(getAngleDegrees(), 0);
        }

        else {
            encoderOffset = 0;
            thoughBoreEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            thoughBoreEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            setTargetPosition(0, 0);
            updateEncoderPosition();
        }
    }


    public void setTargetPosition(double angleDegrees, double slideExtension) {
        VLRSubsystem.getLogger(MainArmSubsystem.class).log(Level.WARNING, "NEW ROTATOR ANGLE OF " + angleDegrees + " JUST SET");
        double target = clamp(angleDegrees, MIN_ANGLE, MAX_ANGLE);
        double maxVelocity = mapToRange(slideExtension, 0, 1, NOMINAL_VELOCITY, EXTENDED_VELOCITY);

        if (Math.abs(angleDegrees - getAngleDegrees()) < 8){
            motionProfile.updateCoefficients(ACCELERATION, ACCELERATION, maxVelocity, PROPORTIONAL, INTEGRAL , DERIVATIVE, 0, 0);
        }
        else{
            motionProfile.updateCoefficients(ACCELERATION, ACCELERATION, maxVelocity, PROPORTIONAL, 0 , DERIVATIVE, 0, 0);
        }
        motionProfile.setTargetPosition(target);
        holdPointController.setTargetPosition(target);
    }


    public double getAngleDegrees() {
        return encoderPosition / ENCODER_TICKS_PER_ROTATION * 360d;
    }

    public boolean reachedTargetPosition() {
        return reachedPosition(motionProfile.getTargetPosition());
    }

    public double getTargetPosition(){
        return motionProfile.getTargetPosition();
    }


    public boolean reachedPosition(double angleDegrees) {
        return Math.abs(getAngleDegrees() - angleDegrees) < ERROR_MARGIN;
    }

    public double getT(){
        return motionProfile.getT();
    }

//    public void updateCoefficientsForOperationMode(){
//        holdPointController.setPID(HOLD_POINT_PROPORTIONAL, 0, HOLD_POINT_DERIVATIVE);
//        if (operationMode == OPERATION_MODE.HANG) {motionProfile.updateCoefficients(0,0 , 0 , 0 ,0 ,0 ,0 , 0);}
//        else {motionProfile.updateCoefficients(ACCELERATION, ACCELERATION, NOMINAL_VELOCITY).setPID(PROPORTIONAL, 0, DERIVATIVE);}
//    }

    public void resetEncoder(){
        encoderOffset = thoughBoreEncoder.getCurrentPosition();
    }

    public void enablePowerOverride(double power){
        System.out.println("ROTATOR POWER OVERRIDE ENABLED");
        powerOverride_state = true;
        powerOverride_value = power;
    }

    public void disablePowerOverride(){
        System.out.println("ROTATOR POWER OVERRIDE DISABLED");
        powerOverride_value = 0;
        powerOverride_state = false;
    }


    private void updateEncoderPosition(){
        encoderPosition = -(thoughBoreEncoder.getCurrentPosition() - encoderOffset);
    }

    public void setPowerLimit(double powerLimit){
        this.powerLimit = powerLimit;
    }


    public void periodic(double slideExtension, OPERATION_MODE operationMode) {
        updateEncoderPosition();
        double currentAngle = getAngleDegrees();
        boolean currentBeamBreakState = breamBreak.isPressed();
        this.operationMode = operationMode;

        if (DEBUG_MODE){
            //updateCoefficientsForOperationMode();
        }

        double feedForward = FEEDFORWARD_GAIN;
        if (operationMode == OPERATION_MODE.HANG){
            feedForward = FEEDFORWARD_GAIN_HANG;
        }
        else{
            feedForward = mapToRange(slideExtension, 0, 1, FEEDFORWARD_GAIN, FEEDFORWARD_GAIN_EXTENDED);
        }

        double feedForwardPower = Math.cos(Math.toRadians(currentAngle)) * feedForward;
        double power = motionProfile.getPower(currentAngle) + feedForwardPower;


        if (operationMode == OPERATION_MODE.HOLD_POINT){
            power = holdPointController.getPower(currentAngle);
            if (motionProfile.getTargetPosition() != 0){
                power += feedForwardPower;
            }
        }

        power = clamp(power, -powerLimit, powerLimit);

        if (currentBeamBreakState && motionProfile.getTargetPosition() == 0) {
            if (!prevBreamBreakState) {timer.reset();}
            else if(timer.seconds() > 2 && !encoderReset) {
                //resetEncoder();
                encoderReset = true;
            }

            if (Math.abs(currentAngle) < 0.2){
                power = 0;
            }
        }
        else {
            encoderReset = false;
        }
        prevBreamBreakState = currentBeamBreakState;

        if (DEBUG_MODE) {
            Telemetry telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());
            telemetry.addData("CURRENT_OPERATION_MODE: ", operationMode);
            //telemetry.addData("ROTATOR_targetPosition: ", motionProfile.getTargetPosition());
            //telemetry.addData("ROTATOR_currentPosition: ", currentAngle);
            //telemetry.addData("ROTATOR T: ", getT());
            telemetry.addData("ROTATOR_power: ", power);
        }
//        Telemetry telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());
//        double motorPowerWatts = motor.getCurrent(CurrentUnit.AMPS) * voltageSensor.getVoltage();
//
//        if (prevTime != 0){
//            double dt = (System.nanoTime() - prevTime) / 10e9;
//            energy += motorPowerWatts * dt;
//        }
//
//        prevTime = System.nanoTime();
//        telemetry.addData("ROTATOR CONSUMED ENERGY SINCE INIT (Ws): ",  energy);

        if (powerOverride_state) {motor.setPower(powerOverride_value);}
        else {motor.setPower(power);}
    }
}