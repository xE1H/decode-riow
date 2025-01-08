package org.firstinspires.ftc.teamcode.subsystems.chassis;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.localization.Localizer;
import org.firstinspires.ftc.teamcode.auto.pedroPathing.localization.localizers.PinpointLocalizer;
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

    public static double motorPower = 1;
    public static double acceleration_a = 0.96;
    public static double deceleration_a = 0.7;

    public static double forwardsMultiplier = 0.85;
    public static double strafeMultiplier = 0.5;


    public static double staticFrictionBar = 0.05;

    AsymmetricLowPassFilter x_filter = new AsymmetricLowPassFilter(acceleration_a, deceleration_a);
    AsymmetricLowPassFilter y_filter = new AsymmetricLowPassFilter(acceleration_a, deceleration_a);

    Localizer localizer;


    @Override
    protected void initialize(HardwareMap hardwareMap) {
        localizer = new PinpointLocalizer(hardwareMap);

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

        MotorRightBack.setInverted(true);
        MotorRightFront.setInverted(true);
    }

    public void drive(Pose2d positionVector) {
        this.driveMotors(new MecanumDriveController(
                strafeMultiplier * x_filter.estimatePower(positionVector.getX()),
                forwardsMultiplier * y_filter.estimatePower(positionVector.getY()),
                (positionVector.getHeading()) * 0.05
        ));
    }

    public void drive(double xSpeed, double ySpeed, double zRotation) {
        this.driveMotors(new MecanumDriveController(xSpeed, ySpeed, zRotation));
    }

    private void driveMotors(MecanumDriveController driveController) {
        driveController.normalize(1.0);


        MotorLeftFront.set(clampPower(driveController.frontLeftMetersPerSecond) * motorPower);
        MotorRightFront.set(clampPower(driveController.frontRightMetersPerSecond) * motorPower);
        MotorLeftBack.set(clampPower(driveController.rearLeftMetersPerSecond) * motorPower);
        MotorRightBack.set(clampPower(driveController.rearRightMetersPerSecond) * motorPower);

        if(GlobalConfig.DEBUG_MODE){
            Telemetry telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());
            telemetry.addData("Motor FL", MotorLeftFront.motorEx.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Motor FR", MotorRightFront.motorEx.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Motor BL", MotorLeftBack.motorEx.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Motor BR", MotorRightBack.motorEx.getCurrent(CurrentUnit.AMPS));
        }


    }

    public double clampPower(double motorPower){
        if (Math.abs(motorPower) < staticFrictionBar) return 0;
        else return motorPower;
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
}