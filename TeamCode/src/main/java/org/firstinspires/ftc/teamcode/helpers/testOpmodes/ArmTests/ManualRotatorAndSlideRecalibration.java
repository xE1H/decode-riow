package org.firstinspires.ftc.teamcode.helpers.testOpmodes.ArmTests;

import static org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorConfiguration.MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideConfiguration.ENCODER_NAME;
import static org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideConfiguration.MOTOR_NAME_0;
import static org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideConfiguration.MOTOR_NAME_1;
import static org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideConfiguration.MOTOR_NAME_2;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorConfiguration;

//@Disabled
@TeleOp(group = "!TELEOP")
public class ManualRotatorAndSlideRecalibration extends OpMode {
    DcMotorEx rotator;
    DcMotorEx slideMotor0, slideMotor1, slideMotor2, slideEncoder, rotatorEncoder;


    @Override
    public void init() {
        rotator = hardwareMap.get(DcMotorEx.class, MOTOR_NAME);
        rotator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor0 = hardwareMap.get(DcMotorEx.class, MOTOR_NAME_0);
        slideMotor1 = hardwareMap.get(DcMotorEx.class, MOTOR_NAME_1);
        slideMotor2 = hardwareMap.get(DcMotorEx.class, MOTOR_NAME_2);
        slideEncoder = hardwareMap.get(DcMotorEx.class, ENCODER_NAME);
        rotatorEncoder = hardwareMap.get(DcMotorEx.class, ArmRotatorConfiguration.ENCODER_NAME);

    }

    @Override
    public void loop() {

        rotator.setPower(gamepad1.right_stick_y);
        double slide = gamepad1.left_stick_y;

        slideMotor0.setPower(slide);
        slideMotor1.setPower(slide);
        //slideMotor2.setPower(slide);
    }

    @Override
    public void stop(){
        rotatorEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
