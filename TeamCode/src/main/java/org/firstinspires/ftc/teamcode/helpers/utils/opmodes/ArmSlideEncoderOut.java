package org.firstinspires.ftc.teamcode.helpers.utils.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideConfiguration;

@TeleOp(name = "ArmSlideEncoderOutput", group = "Utils")
public class ArmSlideEncoderOut extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor motor = hardwareMap.get(DcMotor.class, ArmSlideConfiguration.ENCODER_NAME);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();
        // -433 == 24.5cm
        while (opModeIsActive()) {
            telemetry.addData("Position:", motor.getCurrentPosition());
            telemetry.update();
        }
    }
}
