package org.firstinspires.ftc.teamcode.helpers.utils;

/**
 * The classic SquID controller. Operates off the control law of response being proportional to the (signed) square root of the error.
 */
public class SquIDController {

    /**
     * The proportional gain. This gain multiplies the signed square root of error.
     */
    private double p;
    private double targetPosition = 0;
    private double prevTargetPosition = 0;
    private double t = 1;
    private double startPosition = 0;

    public SquIDController(double p) {
        this.p = p;
    }

    /**
     * The function that computes the intended system response.
     * @param currentPosition The current state of the system.
     * @return The system response.
     */
    public double getPower(double currentPosition) {
        double error = targetPosition - currentPosition;

        if (targetPosition != prevTargetPosition){
            startPosition = currentPosition;
            prevTargetPosition = targetPosition;
        }

        if (targetPosition == startPosition){t = 1;}
        else {t = (currentPosition - startPosition) / (targetPosition - startPosition);}

        return signedSqrt(error) * p;
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

    public void setP(double p) {
        this.p = p;
    }

    /**
     * Signed square root function.
     * @return The square root of the magnitude of <code>val</code>, with the same sign as <code>val</code>.
     */
    private static double signedSqrt(double val) {
        return Math.signum(val) * Math.sqrt(Math.abs(val));
    }
}