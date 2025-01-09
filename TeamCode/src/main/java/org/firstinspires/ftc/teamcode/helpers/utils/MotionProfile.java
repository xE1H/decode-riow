package org.firstinspires.ftc.teamcode.helpers.utils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

// Generic PIDF controller for motion profiling. This class is used to generate a motion profile for
// a given target position.

public class MotionProfile {
    private final Telemetry telemetry;
    private final ElapsedTime timer = new ElapsedTime();

    private double feedForwardGain, velocityGain, accelerationGain;
    private double acceleration, deceleration, maxVelocity;

    private double currentTargetPosition;
    private double prevTargetPosition;
    private double initialPosition;

    private double creep;

    private final String telemetryName;
    private boolean isTelemetryEnabled;

    private final FeedforwardType feedforwardType;
    private final PIDController pid;
    private final PIDController pd;

    private OperationMode operationMode = OperationMode.NOT_USE_INTEGRAL;

    public MotionProfile(Telemetry telemetry, String telemetryName, double acceleration, double deceleration, double maxVelocity, double creep, double feedbackProportionalGain, double feedbackIntegralGain, double feedbackDerivativeGain, double f, double v, double a, FeedforwardType feedforwardType){
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        this.acceleration = acceleration;
        this.deceleration = deceleration;
        this.maxVelocity = maxVelocity;
        this.creep = creep;
        this.feedForwardGain = f;
        this.velocityGain = v;
        this.accelerationGain = a;
        this.feedforwardType = feedforwardType;
        this.telemetryName = telemetryName;
        this.pid = new PIDController(feedbackProportionalGain, feedbackIntegralGain, feedbackDerivativeGain);
        this.pd = new PIDController(feedbackProportionalGain, 0, feedbackDerivativeGain);
    }

    public MotionProfile(Telemetry telemetry, String telemetryName, double acceleration, double deceleration, double maxVelocity, double creep, double p, double i, double d, double v, double a, FeedforwardType feedforwardType){
        this(telemetry, telemetryName, acceleration, deceleration, maxVelocity, creep, p, i, d, 0, v, a, feedforwardType);
    }

    public MotionProfile(Telemetry telemetry, String telemetryName, double acceleration, double deceleration, double maxVelocity, double p, double i, double d, double v, double a, FeedforwardType feedforwardType){
        this(telemetry, telemetryName, acceleration, deceleration, maxVelocity, 0, p, i, d, 0, v, a, feedforwardType);
    }

    public void updateCoefficients(double acceleration, double deceleration, double maxVelocity, double p, double i, double d, double v, double a){
        this.acceleration = acceleration;
        this.deceleration = deceleration;
        this.maxVelocity = maxVelocity;
        this.velocityGain = v;
        this.accelerationGain = a;
        pid.setPID(p, i, d);
    }

    public void setFeedForwardGain(double f){
        this.feedForwardGain = f;
    }

    public void setCurrentTargetPosition(double currentTargetPosition){
        this.currentTargetPosition = currentTargetPosition;
    }

    public double getComputedPositionPower(double currentPos, double targetPos){
        return pid.calculate(currentPos, targetPos);
    }

    public double getPower(double currentPosition, double feedforwardAngle){
        if (currentTargetPosition != prevTargetPosition) {
            prevTargetPosition = currentTargetPosition;
            initialPosition = currentPosition;
            timer.reset();
        }

        double positionError = currentTargetPosition - initialPosition;
        MotionState motionState;

        if (Math.abs(positionError) > creep) motionState = computeMotionState(Math.abs(positionError), timer.seconds());
        else motionState = new MotionState(Math.abs(positionError), 0, 0);

        double positionSetPoint = initialPosition + Math.signum(positionError) * motionState.position;
        double pidPositionPower = pid.calculate(currentPosition, positionSetPoint);
        double pdPositionPower = pd.calculate(currentPosition, positionSetPoint);
        double positionPower = 0;

        switch (operationMode){
            case USE_INTEGRAL:
                positionPower = pidPositionPower;
                break;
            case NOT_USE_INTEGRAL:
                positionPower = pdPositionPower;
        }

        double velocityPower = motionState.velocity * velocityGain * Math.signum(positionError);
        double accelerationPower = motionState.acceleration * accelerationGain * Math.signum(positionError);
        double feedforward = computeFeedforwardPower(feedforwardAngle);

        telemetry.addData(telemetryName + "position", positionPower);
        telemetry.addData(telemetryName + "velocity", velocityPower);
        telemetry.addData(telemetryName + "acceleration", accelerationPower);
        telemetry.addData(telemetryName + "feedforward", feedforward);



        if (isTelemetryEnabled) {
            logTelemetry(
                    currentPosition,
                    feedforwardAngle,
                    positionSetPoint,
                    motionState,
                    positionPower,
                    velocityPower,
                    accelerationPower
            );
        }

        return positionPower + velocityPower + accelerationPower + feedforward;
    }


    public double getPower(double currentAngle){
        return getPower(currentAngle, currentAngle);
    }

    private double computeFeedforwardPower(double feedforwardAngle){
        switch (feedforwardType){
            case SINE:
                return Math.sin(Math.toRadians(feedforwardAngle)) * feedForwardGain;
            case COSINE:
                return Math.cos(Math.toRadians(feedforwardAngle)) * feedForwardGain;
            case CONSTANT:
                return feedForwardGain;
            case NO_FEEDFORWARD:
                return 0;
        }
        return 0;
    }

    private MotionState computeMotionState(double distance, double elapsed_time) {
        if (distance == 0) {
            return new MotionState(0, 0, 0);
        }
        // Calculate the time it takes to accelerate to max velocity
        double acceleration_dt = maxVelocity / acceleration;
        double deceleration_dt = maxVelocity / deceleration;

        // If we can't accelerate to max velocity in the given distance, we'll accelerate as much as possible
        double acceleration_distance = 0.5 * acceleration * Math.pow(acceleration_dt, 2);
        double deceleration_distance = 0.5 * deceleration * Math.pow(deceleration_dt, 2);

        if (acceleration_distance + deceleration_distance > distance) {
            acceleration_dt = Math.sqrt(2 * distance / (acceleration * (1 + acceleration / deceleration)));
        }

        acceleration_distance = 0.5 * acceleration * Math.pow(acceleration_dt, 2);

        // recalculate max velocity based on the time we have to accelerate and decelerate
        double maxAchievableVelocity = acceleration * acceleration_dt;

        deceleration_dt = maxAchievableVelocity / deceleration;
        deceleration_distance = 0.5 * deceleration * Math.pow(deceleration_dt, 2);

        // calculate the time that we're at max velocity
        double cruise_distance = distance - acceleration_distance - deceleration_distance;
        double cruise_dt = cruise_distance / maxAchievableVelocity;
        double deceleration_time = acceleration_dt + cruise_dt;

        // check if we're still in the motion profile
        double entire_dt = acceleration_dt + cruise_dt + deceleration_dt;
        if (elapsed_time > entire_dt) {
            return new MotionState(distance, 0, 0);
        }

        // if we're accelerating
        if (elapsed_time < acceleration_dt) {
            return new MotionState(0.5 * acceleration * Math.pow(elapsed_time, 2), acceleration * elapsed_time, acceleration);
        }

        // if we're cruising
        else if (elapsed_time < deceleration_time) {
            acceleration_distance = 0.5 * acceleration * Math.pow(acceleration_dt, 2);
            double cruise_current_dt = elapsed_time - acceleration_dt;

            return new MotionState(acceleration_distance + maxAchievableVelocity * cruise_current_dt, maxAchievableVelocity, 0);
        }

        // if we're decelerating
        else {
            acceleration_distance = 0.5 * acceleration * Math.pow(acceleration_dt, 2);
            cruise_distance = maxAchievableVelocity * cruise_dt;
            deceleration_time = elapsed_time - deceleration_time;

            return new MotionState(acceleration_distance + cruise_distance + maxAchievableVelocity * deceleration_time - 0.5 * deceleration * Math.pow(deceleration_time, 2),
                    maxAchievableVelocity - deceleration * deceleration_time, deceleration);
        }
    }

    public void enableTelemetry(boolean enableTelemetry){
        this.isTelemetryEnabled = enableTelemetry;
    }


    public void logTelemetry(
            double currentPosition,
            double feedforwardAngle,
            double positionSetPoint,
            MotionState motionState,
            double positionPower,
            double velocityPower,
            double accelerationPower
    ) {
        logTelemetry(
                currentPosition,
                feedforwardAngle,
                positionSetPoint,
                motionState.velocity,
                motionState.acceleration,
                positionPower,
                velocityPower,
                accelerationPower
        );
    }

    public void logTelemetry(
            double currentPosition,
            double feedforwardAngle,
            double positionSetPoint,
            double velocitySetPoint,
            double accelerationSetPoint,
            double positionPower,
            double velocityPower,
            double accelerationPower
    ){
        telemetry.addData(telemetryName + "_motionProfileCurrentPosition: ", currentPosition);
        telemetry.addData(telemetryName + "_motionProfileFeedforwardAngle: ", feedforwardAngle);
        telemetry.addData(telemetryName + "_motionProfileTargetPosition: ", positionSetPoint);
        telemetry.addData(telemetryName + "_motionProfileTargetVelocity: ", velocitySetPoint);
        telemetry.addData(telemetryName + "_motionProfileTargetAcceleration: ", accelerationSetPoint);
        telemetry.addData(telemetryName + "_motionProfileTime: ", timer.seconds());
        telemetry.addData(telemetryName + "_motor power: ", positionPower + velocityPower + accelerationPower);
        telemetry.update();
    }

    public double getTargetPosition() {
        return currentTargetPosition;
    }

    public enum FeedforwardType{
        SINE,
        COSINE,
        CONSTANT,
        NO_FEEDFORWARD
    }

    public enum OperationMode{
        USE_INTEGRAL,
        NOT_USE_INTEGRAL
    }

    public void setOperationMode (OperationMode operationMode){
        this.operationMode = operationMode;
    }

    public OperationMode getOperationMode(){
        return operationMode;
    }
}