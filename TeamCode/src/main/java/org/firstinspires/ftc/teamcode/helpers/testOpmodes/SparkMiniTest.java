package org.firstinspires.ftc.teamcode.helpers.testOpmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;


@Config
@Disabled
@TeleOp(name="SPARK MINI TEST", group="Utils")
public class SparkMiniTest extends OpMode {
    private CRServo spark;

    //REVERSE ROTATOR, NOT SLIDE MOTORS

    @Override
    public void init() {
        spark = hardwareMap.get(CRServo.class, "MotorArm1");
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        spark.setPower(gamepad1.left_stick_y);
        telemetry.addData("power", gamepad1.left_stick_y);
        System.out.println("power: " + gamepad1.left_stick_y);
    }
}
