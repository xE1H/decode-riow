package org.firstinspires.ftc.teamcode.subsystems.hang;

import com.ThermalEquilibrium.homeostasis.Filters.FilterAlgorithms.LowPassFilter;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;

@Config
public class HangSubsystem extends VLRSubsystem<HangSubsystem> implements HangConfiguration {
    private CRServo left, right;
    private AnalogInput analogLeft, analogRight;

    private LowPassFilter leftFilter = new LowPassFilter(0.93);
    private LowPassFilter rightFilter = new LowPassFilter(0.93);


    @Override
    protected void initialize(HardwareMap hardwareMap) {
        left = hardwareMap.get(CRServo.class, LEFT_AXON);
        right = hardwareMap.get(CRServo.class, RIGHT_AXON);

        left.setDirection(DcMotorSimple.Direction.FORWARD);
        right.setDirection(DcMotorSimple.Direction. REVERSE);

        analogLeft = hardwareMap.get(AnalogInput.class, LEFT_ANALOG);
        analogRight = hardwareMap.get(AnalogInput.class, RIGHT_ANALOG);
    }


    @Override
    public void periodic(){
        Telemetry telemetry = FtcDashboard.getInstance().getTelemetry();

        telemetry.addData("HANG_LEFT_ANALOG_ANGLE: ", getAngle(analogLeft.getVoltage()));
        telemetry.addData("HANG_RIGHT_ANALOG_ANGLE: ", getAngle(analogRight.getVoltage()));
        telemetry.addData("HANG_ANALOG_THRESHOLD_STATE: ", analogFeedbackThresholdReached() ? 1 : 0);
    }


    public void setPower(double power){
        left.setPower(power);
        right.setPower(power);
    }


    private double getAngle(double voltage){
        return voltage / 3.3 * 360;
    }


    public boolean analogFeedbackThresholdReached(){
        return (
                leftFilter.estimate(getAngle(analogLeft.getVoltage())) > leftAnalogThreshold &&
                rightFilter.estimate(getAngle(analogRight.getVoltage())) < rightAnalogThreshold
        );
    }
}