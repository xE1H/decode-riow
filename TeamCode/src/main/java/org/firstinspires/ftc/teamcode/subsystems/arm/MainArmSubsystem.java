package org.firstinspires.ftc.teamcode.subsystems.arm;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;
import static org.firstinspires.ftc.teamcode.subsystems.arm.MainArmConfiguration.*;
import static org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideConfiguration.MAX_EXTENSION_CM;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideSubsystem;

import java.util.logging.Level;

public class MainArmSubsystem extends VLRSubsystem<MainArmSubsystem>{
    private ArmRotatorSubsystem rotator;
    private ArmSlideSubsystem slides;

    private Point targetPosition = new Point(0, 0);
    private Point prevTargetPosition = new Point(0, 0);

    private boolean interpolation = false;
    private ElapsedTime interpolationTimer = new ElapsedTime();


    protected void initialize(HardwareMap hardwareMap){
        rotator = new ArmRotatorSubsystem(hardwareMap);
        slides = new ArmSlideSubsystem(hardwareMap);
    }

    public void setTargetPoint(double magnitude, double angleDegrees) {
        targetPosition = new Point(magnitude, angleDegrees, Point.POLAR);
    }

    public void setTargetPointWithRealWordCoordinates(double x_cm, double y_cm, OFFSET_REFERENCE_PLANE reference){
        Vector2d endEffectorFromPivotReferencePoint = new Vector2d(
                reference.xScalar * (x_cm + ROBOT_LENGTH_CM / 2 + ARM_PIVOT_POINT_OFFSET_FROM_ROBOT_CENTER.getX()),
                y_cm - ARM_PIVOT_POINT_OFFSET_FROM_ROBOT_CENTER.getY()
        );

        double alpha = Math.toDegrees(Math.acos(RETRACTED_END_EFFECTOR_OFFSET_FROM_PIVOT_POINT.getY() / endEffectorFromPivotReferencePoint.magnitude()));
        Vector2d retractedArmEndFactor = RETRACTED_END_EFFECTOR_OFFSET_FROM_PIVOT_POINT.rotateBy(90 + Math.toDegrees(endEffectorFromPivotReferencePoint.angle()) - alpha);

        Vector2d extensionVector = endEffectorFromPivotReferencePoint.minus(retractedArmEndFactor);
        extensionVector = extensionVector.scale(1 / MAX_EXTENSION_CM);

        targetPosition = new Point(extensionVector.getX(), extensionVector.getY(), Point.CARTESIAN);
    }


    public void setTargetExtension(double extension) {setTargetPoint(extension, getTargetAngleRads());}

    public void setTargetAngle(double angleRads) {setTargetPoint(getTargetExtension(), angleRads);}

    public void moveTargetRelative(double deltaX_cm, double deltaY_cm, OFFSET_REFERENCE_PLANE reference) {setTargetPointWithRealWordCoordinates(targetPosition.getX() + deltaX_cm, targetPosition.getY() + deltaY_cm, reference);}

    public Point getTargetPoint() {return targetPosition;}

    public double getTargetAngleDegrees() {return clamp(Math.toDegrees(targetPosition.getTheta()), 0, 180);}

    public double getTargetAngleRads() {return clamp(targetPosition.getTheta(), 0, Math.PI);}

    public double getTargetExtension() {return clamp(targetPosition.getR(), 0, 1);}

    public double getTargetX() {return clamp(targetPosition.getX(), -1, 1);}

    public  double getTargetY() {return clamp(targetPosition.getY(), 0, 1);}

    public void setInterpolation(boolean interpolation) {
        this.interpolation = interpolation;
        interpolationTimer.reset();
    }


    @Override
    public void periodic(){
        if (interpolation) {
            double delta = Math.hypot(targetPosition.getX() - prevTargetPosition.getX(), targetPosition.getY() - prevTargetPosition.getY());
            double currentScalar = clamp(interpolationTimer.seconds() * delta / interpolationTimeConstant, 0 ,1);

            if (currentScalar < 1){
                double currentX = prevTargetPosition.getX() + (targetPosition.getX() - prevTargetPosition.getX()) * currentScalar;
                double currentY = prevTargetPosition.getY() + (targetPosition.getY() - prevTargetPosition.getY()) * currentScalar;

                Point interpolatedTarget = new Point(currentX, currentY);

                rotator.setTargetPosition(Math.toDegrees(interpolatedTarget.getTheta()), interpolatedTarget.getR());
                slides.setTargetPosition(interpolatedTarget.getR());
            }
            else if (currentScalar == 1 && prevTargetPosition != targetPosition){
                rotator.setTargetPosition(Math.toDegrees(targetPosition.getTheta()), targetPosition.getR());
                slides.setTargetPosition(targetPosition.getR());
                prevTargetPosition = targetPosition;
            }
        }


        else if (prevTargetPosition != targetPosition){
            rotator.setTargetPosition(Math.toDegrees(targetPosition.getTheta()), targetPosition.getR());
            slides.setTargetPosition(targetPosition.getR());
            prevTargetPosition = targetPosition;
        }


        logger.log(Level.INFO, "TARGET ANGLE FROM ARM SUBSYSTEM: " + Math.toDegrees(targetPosition.getTheta()));
        logger.log(Level.INFO, "TARGET EXTENSION FROM ARM SUBSYSTEM: " + targetPosition.getR());

        rotator.periodic(Math.toDegrees(targetPosition.getTheta()));
        slides.periodic(targetPosition.getR());
    }


    public double coordinatesToTheta(double magnitudeOrX, double thetaOrY, COORDINATE_IDENTIFIER coordinateIdentifier){
        if (coordinateIdentifier == COORDINATE_IDENTIFIER.POLAR) {return thetaOrY;}
        else {return Math.atan2(thetaOrY, magnitudeOrX);}
    }

    public double coordinatesToExtension(double magnitudeOrX, double thetaOrY, COORDINATE_IDENTIFIER coordinateIdentifier){
        if (coordinateIdentifier == COORDINATE_IDENTIFIER.POLAR) {return magnitudeOrX;}
        else {return Math.hypot(magnitudeOrX, thetaOrY);}
    }

    public boolean isBetween(double num, double num1, double num2){
        return num > Math.min(num1, num2) && num < Math.max(num1, num2);
    }

    public boolean isTargetPointValid(double targetAngle){
        return !isBetween(targetAngle, EXCLUSION_ZONE_MIN_ANGLE, EXCLUSION_ZONE_MAX_ANGLE);
    }

    public boolean isNewTargetSafeToMoveDirectly(double targetAngle){
        double prevTarget = Math.toDegrees(prevTargetPosition.getTheta());
        return (isBetween(EXCLUSION_ZONE_MIN_ANGLE, prevTarget, targetAngle) || isBetween(EXCLUSION_ZONE_MAX_ANGLE, prevTarget, targetAngle)) && prevTargetPosition.getR() > EXCLUSION_ZONE_MIN_EXTENSION;
    }

    public boolean reachedTargetPosition(){
        return rotator.reachedTargetPosition() && slides.reachedTargetPosition();
    }

    public boolean motionProfilePathsAtParametricEnd(){
        return rotator.getT() == 1 && slides.getT() == 1;
    }
}