package org.firstinspires.ftc.teamcode.helpers.testOpmodes;

import static org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration.GRAB_SERVO;

import com.acmerobotics.dashboard.config.Config;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "AngleServoCalibration", group = "Utils")
@Photon
@Config
public class AngleServoCalibration extends OpMode {
    Servo servo, servo1;

    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, GRAB_SERVO);
    }

    @Override
    public void loop() {
        servo.setPosition(gamepad1.right_stick_y);
        telemetry.addData("servo pos: ", gamepad1.right_stick_y);
    }
}
