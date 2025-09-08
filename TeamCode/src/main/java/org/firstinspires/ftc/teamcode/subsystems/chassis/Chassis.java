package org.firstinspires.ftc.teamcode.subsystems.chassis;

import static org.firstinspires.ftc.teamcode.helpers.utils.GlobalConfig.DEBUG_MODE;

import com.ThermalEquilibrium.homeostasis.Filters.FilterAlgorithms.LowPassFilter;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.chassis.helpers.AsymmetricLowPassFilter;
import org.firstinspires.ftc.teamcode.subsystems.chassis.helpers.MecanumDriveController;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

@Config
public class Chassis extends VLRSubsystem<Chassis> implements ChassisConfiguration {
    private static final Logger log = LoggerFactory.getLogger(Chassis.class);
    MotorEx MotorLeftFront;
    MotorEx MotorRightFront;
    MotorEx MotorLeftBack;
    MotorEx MotorRightBack;

    public static double motorPower = 1;
    public static double acceleration_a = 0.4;
    public static double deceleration_a = 0.2;

    public static double forwardsMultiplier = 1;
    public static double strafeMultiplier = 0.8;

    public static double staticFrictionBar = 0.05;

    boolean isDriveFieldCentric = false;

    AsymmetricLowPassFilter x_filter = new AsymmetricLowPassFilter(acceleration_a, deceleration_a);
    AsymmetricLowPassFilter y_filter = new AsymmetricLowPassFilter(acceleration_a, deceleration_a);

    @Override
    protected void initialize(HardwareMap hardwareMap) {

        MotorLeftFront = new MotorEx(hardwareMap, MOTOR_LEFT_FRONT);
        MotorRightFront = new MotorEx(hardwareMap, MOTOR_RIGHT_FRONT);
        MotorLeftBack = new MotorEx(hardwareMap, MOTOR_LEFT_BACK);
        MotorRightBack = new MotorEx(hardwareMap, MOTOR_RIGHT_BACK);

        setMotorsToBrake();

        MotorLeftBack.setRunMode(Motor.RunMode.RawPower);
        MotorRightBack.setRunMode(Motor.RunMode.RawPower);
        MotorRightFront.setRunMode(Motor.RunMode.RawPower);
        MotorLeftFront.setRunMode(Motor.RunMode.RawPower);

//        MotorRightBack.setInverted(true);
//        MotorRightFront.setInverted(true);
    }


    public void enableRobotCentric() {
        isDriveFieldCentric = false;
    }


    public void enableFieldCentric() {
        isDriveFieldCentric = true;
    }

//    public void drive(Pose2d positionVector) {
//        localizer.update();
//        this.driveMotors(new MecanumDriveController(
//                strafeMultiplier * x_filter.estimatePower(positionVector.getX()),
//                forwardsMultiplier * y_filter.estimatePower(positionVector.getY()),
//                (positionVector.getHeading()) * 0.05
//        ));
//    }

    public void drive(double xSpeed, double ySpeed, double zRotation) {
        // This sometimes fails and causes the whole robot to die
        Vector2d vector = new Vector2d(x_filter.estimatePower(xSpeed) * forwardsMultiplier, y_filter.estimatePower(ySpeed) * strafeMultiplier);
        this.driveMotors(new MecanumDriveController(vector.getX(), vector.getY(), zRotation));
    }

    private void driveMotors(MecanumDriveController driveController) {
        driveController.normalize(1.0);

        MotorLeftFront.set(clampPower(driveController.frontLeftMetersPerSecond) * motorPower);
        MotorRightFront.set(clampPower(driveController.frontRightMetersPerSecond) * motorPower);
        MotorLeftBack.set(clampPower(driveController.rearLeftMetersPerSecond) * motorPower);
        MotorRightBack.set(clampPower(driveController.rearRightMetersPerSecond) * motorPower);
    }

    public double clampPower(double motorPower) {
        if (Math.abs(motorPower) < staticFrictionBar) return 0;
        return motorPower;
    }

    public void stop() {
        MotorLeftFront.stopMotor();
        MotorRightFront.stopMotor();
        MotorLeftBack.stopMotor();
        MotorRightBack.stopMotor();

        motorPower = 0.0;
    }

    public void setMotorsToBrake(){
        MotorLeftFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        MotorRightFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        MotorLeftBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        MotorRightBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    public void setPower(double power) {
        motorPower = Math.min(power, 1.0);
    }
}