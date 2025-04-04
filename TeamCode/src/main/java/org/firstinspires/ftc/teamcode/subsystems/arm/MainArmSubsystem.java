package org.firstinspires.ftc.teamcode.subsystems.arm;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideSubsystem;

@Config
public class MainArmSubsystem extends VLRSubsystem<MainArmSubsystem>{
    private ArmRotatorSubsystem rotator;
    private ArmSlideSubsystem slides;

    public static final int POLAR = 0;
    public static final int CARTESIAN = 1;

    private Point targetPosition = new Point(0, 0);
    private Point prevTargetPosition = new Point(0, 0);

    private boolean interpolate = false;
    public static double interpolationSpeed = 0;
    private ElapsedTime interpolationTimer = new ElapsedTime();


    protected void initialize(HardwareMap hardwareMap){
        rotator = new ArmRotatorSubsystem(hardwareMap);
        slides = new ArmSlideSubsystem(hardwareMap);
    }

    public void setTargetPoint(double magnitudeOrX, double thetaOrY, int identifier) {
        prevTargetPosition = targetPosition;
        interpolate = false;
        targetPosition = new Point(magnitudeOrX, thetaOrY, identifier);
    }

    public void setInterpolatedTargetPoint(double magnitudeOrX, double thetaOrY, int identifier) {
        setTargetPoint(magnitudeOrX, thetaOrY, identifier);
        interpolate = true;
    }

    public Point getTargetPoint() {return targetPosition;}

    public double getTargetAngleDegrees() {return Math.toDegrees(targetPosition.getTheta());}

    public double getTargetAngleRads() {return targetPosition.getTheta();}

    public double getTargetExtension() {return targetPosition.getR();}

    public double getTargetX() {return targetPosition.getX();}

    public double getTargetY() {return targetPosition.getY();}


    @Override
    public void periodic(){
        if (interpolate) {
            double currentScalar = clamp(interpolationTimer.seconds() * interpolationSpeed, 0, 1);

            double currentX = prevTargetPosition.getX() + (targetPosition.getX() - prevTargetPosition.getX()) * currentScalar;
            double currentY = prevTargetPosition.getY() + (targetPosition.getY() - prevTargetPosition.getY()) * currentScalar;

            Point interpolatedTarget = new Point(currentX, currentY);

            rotator.setTargetPosition(Math.toDegrees(interpolatedTarget.getTheta()));
            slides.setTargetPosition(interpolatedTarget.getR());
        }

        else{
            rotator.setTargetPosition(Math.toDegrees(targetPosition.getTheta()));
            slides.setTargetPosition(targetPosition.getR());
        }
    }
}