package org.firstinspires.ftc.teamcode.subsystems.hang;

import com.ThermalEquilibrium.homeostasis.Filters.FilterAlgorithms.LowPassFilter;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;

@Config
public class HangSubsystem extends VLRSubsystem<HangSubsystem> implements HangConfiguration {
    private CRServo left, right;
    private AnalogInput analogLeft, analogRight;
    private double prevTime = 0;

    private double prevLeftAngle = 0;
    private double prevRightAngle = 0;

    private LowPassFilter leftFilter = new LowPassFilter(0.9);
    private LowPassFilter rightFilter = new LowPassFilter(0.9);

    private DataBuffer leftBuffer, rightBuffer;

    private boolean leftVelocityReached = false;
    private boolean rightVelocityReached = false;

    private boolean moving = false;


    @Override
    protected void initialize(HardwareMap hardwareMap) {
        left = hardwareMap.get(CRServo.class, LEFT_AXON);
        right = hardwareMap.get(CRServo.class, RIGHT_AXON);

        analogLeft = hardwareMap.get(AnalogInput.class, LEFT_ANALOG);
        analogRight = hardwareMap.get(AnalogInput.class, RIGHT_ANALOG);

        left.setDirection(DcMotorSimple.Direction.REVERSE);
        right.setDirection(DcMotorSimple.Direction.REVERSE);

        leftBuffer = new DataBuffer();
        rightBuffer = new DataBuffer();
    }


    @Override
    public void periodic(){
//        double currentTime = System.nanoTime();
//        double dt = (currentTime - prevTime) / Math.pow(10, 9);
//        prevTime = currentTime;
//
//        double currentLeftAngle = getAngle(analogLeft.getVoltage());
//        double velocityLeft = leftFilter.estimate((currentLeftAngle - prevLeftAngle) / dt);
//        prevLeftAngle = currentLeftAngle;
//
//        double currentRightAngle = getAngle(analogRight.getVoltage());
//        double velocityRight = rightFilter.estimate((currentRightAngle - prevRightAngle) / dt);
//        prevRightAngle = currentRightAngle;
//
//
//        if (velocityLeft > velocityThreshold && !moving) leftVelocityReached = true;
//        if (velocityRight > velocityThreshold && !moving) rightVelocityReached = true;

        double leftDelta = leftBuffer.getDelta(getAngle(analogLeft.getVoltage()));
        double rightDelta = rightBuffer.getDelta(getAngle(analogRight.getVoltage()));

        if (leftDelta > positionDelta) leftVelocityReached = true;
        if (rightDelta > positionDelta) rightVelocityReached = true;


        Telemetry telemetry = FtcDashboard.getInstance().getTelemetry();
        telemetry.addData("HANG_delta left: ", leftDelta);
        telemetry.addData("HANG_delta right: ", rightDelta);
        telemetry.addData("HANG_THRESHOLD_REACHED: ", analogFeedbackThresholdReached() ? 1 : 0);
    }


    public void resetMovedConditions(){
        leftVelocityReached = false;
        rightVelocityReached = false;
    }


    public void setMoving(boolean moving){
        this.moving = moving;
    }


    public void setPower(double power) {
        left.setPower(power);
        right.setPower(power);
    }



    private double getAngle(double voltage){
        return voltage / 3.3 * 360;
    }


    public boolean analogFeedbackThresholdReached(){
        return leftVelocityReached;
    }
}