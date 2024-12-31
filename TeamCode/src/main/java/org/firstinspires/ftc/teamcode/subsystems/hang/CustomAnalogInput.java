package org.firstinspires.ftc.teamcode.subsystems.hang;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class CustomAnalogInput {
    private AnalogInput analogInput;
    private Direction direction = Direction.FORWARD;
    private double offset = 0;
    private double prevRotation = 0;
    private double rotations = 0;

    public CustomAnalogInput(HardwareMap hardwareMap, String name) {
        analogInput = hardwareMap.get(AnalogInput.class, name);
    }

    public void setDirection(Direction direction){
        this.direction = direction;
    }


    public void reset(){
        offset = analogInput.getVoltage() / 3.3;
        prevRotation = 0;
    }


    public double getRotation(){
        double rotation = analogInput.getVoltage() / 3.3 - offset;

        if (direction == Direction.REVERSE) rotation *= -1;
        if (Math.abs(rotation - prevRotation) >= 0.5) rotations -= Math.signum(rotation - prevRotation);

        prevRotation = rotation;
        return rotation + rotations;
    }


    public double getAngleDegrees(){
        return getRotation() * 360;
    }

    public double getAngleRads(){
        return Math.toRadians(getAngleDegrees());
    }

    public enum Direction{
        FORWARD,
        REVERSE
    }
}