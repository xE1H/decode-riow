package org.firstinspires.ftc.teamcode.helpers.utils.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleRevHub;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleServo;


@Config
//@Disabled
@TeleOp(group = "Utils")
public class ManualRotatorAndSlideRecalibration extends OpMode {
    DcMotorEx rotator;
    DcMotorEx slideMotor;


    @Override
    public void init() {
        rotator = hardwareMap.get(DcMotorEx.class, "MotorRotator");
        slideMotor = hardwareMap.get(DcMotorEx.class, "ArmMotor1");
    }

    @Override
    public void loop() {

        rotator.setPower(gamepad1.right_stick_y);
        slideMotor.setPower(gamepad1.left_stick_y);

    }
}
