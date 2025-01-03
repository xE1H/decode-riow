package org.firstinspires.ftc.teamcode.subsystems.hang;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;

@Config
public class HangSubsystem extends VLRSubsystem<HangSubsystem> implements HangConfiguration {
    private CRServo leftAxon, rightAxon;
    private CustomAnalogInput analogLeft, analogRight;

    private double targetPosition = TargetPosition.DOWN.rotations;
    private boolean enabled = false;

    public static double k_sync = 0.1;
    public static double rotationErrorMargin = 0.05;
    public static double hold_feedforward = 0;
    public static double hold_proportional = 0.5;

    Telemetry telemetry;



    public void setTargetPosition (TargetPosition position){
        targetPosition = position.rotations;
    }


    protected void initialize(HardwareMap hardwareMap) {
        leftAxon = hardwareMap.get(CRServo.class, LEFT_AXON);
        rightAxon = hardwareMap.get(CRServo.class, RIGHT_AXON);

        leftAxon.setDirection(DcMotorSimple.Direction.FORWARD);
        rightAxon.setDirection(DcMotorSimple.Direction.REVERSE);

        analogLeft = new CustomAnalogInput(hardwareMap, ANALOG_ENCODER_LEFT);
        analogRight = new CustomAnalogInput(hardwareMap, ANALOG_ENCODER_RIGHT);

        analogLeft.setDirection(CustomAnalogInput.Direction.REVERSE);
        analogRight.setDirection(CustomAnalogInput.Direction.FORWARD);

        analogLeft.reset();
        analogRight.reset();
    }


    public void initTelemetry(Telemetry telemetry){
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }


    @Override
    public void periodic() {
        if (enabled){
            double[] powers = getNormalisedPowers();

            leftAxon.setPower(powers[0]);
            rightAxon.setPower(powers[1]);
        }
        else{
            targetPosition = (analogLeft.getRotation() + analogRight.getRotation()) / 2;
        }

        telemetry.addData("HANG_leftRotation: ", analogLeft.getRotation());
        telemetry.addData("HANG_rightRotation: ", analogRight.getRotation());
        telemetry.update();
    }


    private double getPower (double targetRotation, double currentRotation){
        if (Math.abs(targetRotation - currentRotation) < rotationErrorMargin){
            return hold_feedforward + (targetRotation - currentRotation) * hold_proportional;
        }
        else if (targetRotation > currentRotation) return 1;
        else return - 1;
    }


    private double[] getNormalisedPowers(){
        double leftRotation = analogLeft.getRotation();
        double rightRotation = analogRight.getRotation();

        double error = leftRotation - rightRotation;
        double[] powers = {getPower(targetPosition, leftRotation) + k_sync * error,
                           getPower(targetPosition, rightRotation) - k_sync * error};

        double maxPower = 1;
        for (double power : powers){
            if (Math.abs(power) > maxPower) maxPower = Math.abs(power);
        }
        for (int i = 0; i < powers.length - 1; i++) powers[i] /= maxPower;

        return powers;
    }


    public void enable() {enabled = true;}
    public void disable() {enabled = false;}


    public void setLeftPower(double power) {leftAxon.setPower(power);}
    public void setRightPower(double power) {rightAxon.setPower(power);}
}