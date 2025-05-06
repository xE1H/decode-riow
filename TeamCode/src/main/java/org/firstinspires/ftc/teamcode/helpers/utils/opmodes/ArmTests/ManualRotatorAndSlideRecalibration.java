package org.firstinspires.ftc.teamcode.helpers.utils.opmodes.ArmTests;

import static org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorConfiguration.MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideConfiguration.MOTOR_NAME_0;
import static org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideConfiguration.MOTOR_NAME_1;
import static org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideConfiguration.MOTOR_NAME_2;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

//@Disabled
@TeleOp(group = "Utils")
public class ManualRotatorAndSlideRecalibration extends OpMode {
    DcMotorEx rotator;
    DcMotorEx slideMotor0, slideMotor1, slideMotor2;


    @Override
    public void init() {
        rotator = hardwareMap.get(DcMotorEx.class, MOTOR_NAME);
        rotator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor0 = hardwareMap.get(DcMotorEx.class, MOTOR_NAME_0);
        slideMotor1 = hardwareMap.get(DcMotorEx.class, MOTOR_NAME_1);
        slideMotor2 = hardwareMap.get(DcMotorEx.class, MOTOR_NAME_2);

    }

    @Override
    public void loop() {

        rotator.setPower(gamepad1.right_stick_y);
        double slide = gamepad1.left_stick_y;

        slideMotor0.setPower(slide);
        slideMotor1.setPower(slide);
        //slideMotor2.setPower(slide);
    }
}
