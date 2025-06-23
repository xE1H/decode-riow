package org.firstinspires.ftc.teamcode.helpers.utils;


import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PidController {
    private double targetPosition = 0;
    private double prevTargetPosition = 0;
    private double t = 1;
    private double startPosition = 0;
    private final PIDController pid;


    public PidController(double p, double i, double d) {
        pid = new PIDController(p, i, d);
    }

    public double getPower(double currentPosition) {
        if (targetPosition != prevTargetPosition){
            startPosition = currentPosition;
            prevTargetPosition = targetPosition;
        }

        if (targetPosition == startPosition){t = 1;}
        else {t = (currentPosition - startPosition) / (targetPosition - startPosition);}

        return pid.calculate(currentPosition, targetPosition);
    }

    public double getT(){
        return t;
    }

    public void setTargetPosition(double targetPosition){
        this.targetPosition = targetPosition;
    }

    public double getTargetPosition(){
        return targetPosition;
    }

    public void setPID(double p, double i, double d) {
        pid.setPID(p, i, d);
    }
}