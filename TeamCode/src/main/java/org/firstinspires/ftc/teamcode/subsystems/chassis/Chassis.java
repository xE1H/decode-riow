package org.firstinspires.ftc.teamcode.subsystems.chassis;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.helpers.utils.GlobalConfig;
import org.firstinspires.ftc.teamcode.subsystems.chassis.helpers.AsymmetricLowPassFilter;
import org.firstinspires.ftc.teamcode.subsystems.chassis.helpers.MecanumDriveController;

@Config
public class Chassis extends VLRSubsystem<Chassis> implements ChassisConfiguration {
    MotorEx MotorLeftFront;
    MotorEx MotorRightFront;
    MotorEx MotorLeftBack;
    MotorEx MotorRightBack;

    AnalogInput leftAngledSensor;
    AnalogInput rightAngledSensor;

    public static double motorPower = 1;
    public static double acceleration_a = 0.9;
    public static double deceleration_a = 0.6;

    public static double forwardsMultiplier = 0.95;
    public static double strafeMultiplier = 0.7;

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

        MotorLeftFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        MotorRightFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        MotorLeftBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        MotorRightBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        MotorLeftBack.setRunMode(Motor.RunMode.RawPower);
        MotorRightBack.setRunMode(Motor.RunMode.RawPower);
        MotorRightFront.setRunMode(Motor.RunMode.RawPower);
        MotorLeftFront.setRunMode(Motor.RunMode.RawPower);

        leftAngledSensor = hardwareMap.get(AnalogInput.class, LEFT_ANGLED_SENSOR);
        rightAngledSensor = hardwareMap.get(AnalogInput.class, RIGHT_ANGLED_SENSOR);

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

//        if(GlobalConfig.DEBUG_MODE){
//            Telemetry telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());
//            telemetry.addData("Motor FL", MotorLeftFront.motorEx.getCurrent(CurrentUnit.AMPS));
//            telemetry.addData("Motor FR", MotorRightFront.motorEx.getCurrent(CurrentUnit.AMPS));
//            telemetry.addData("Motor BL", MotorLeftBack.motorEx.getCurrent(CurrentUnit.AMPS));
//            telemetry.addData("Motor BR", MotorRightBack.motorEx.getCurrent(CurrentUnit.AMPS));
//        }
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

    public void setPower(double power) {
        motorPower = Math.min(power, 1.0);
    }


    private double getDistanceMM(double voltage){
        return voltage / 3.3 * 1500;
    }


    public Pose calculateRobotPoseFromDistanceSensors(Follower follower){
        double robotAngle = follower.getPose().getHeading();
        double X_offset = getDistanceMM(leftAngledSensor.getVoltage()) + 0.5 * DISTANCE_BETWEEN_ANGLED_SENSORS_MM * Math.sin(robotAngle);
        double Y_offset = getDistanceMM(rightAngledSensor.getVoltage()) + 0.5 * DISTANCE_BETWEEN_ANGLED_SENSORS_MM * Math.cos(robotAngle);

        Pose midpointBetweenSensors = new Pose(BUCKET_CORNER.getX() + X_offset, BUCKET_CORNER.getY() - Y_offset, robotAngle);
        Vector2d rotatedOffset = OFFSET_FROM_SENSOR_MIDPOINT_TO_PEDRO_CENTER.rotateBy(Math.toDegrees(robotAngle));

        return new Pose(midpointBetweenSensors.getX() + rotatedOffset.getX(), midpointBetweenSensors.getY() + rotatedOffset.getY(), robotAngle);
    }
}