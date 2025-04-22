package org.firstinspires.ftc.teamcode.helpers.utils.opmodes;

import static org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration.ANGLE_SERVO;
import static org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration.TWIST_SERVO;

import com.acmerobotics.dashboard.config.Config;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "AngleServoCalibration", group = "Utils")
@Photon
@Config
public class AngleServoCalibration extends OpMode {
    Servo servo;

    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, ANGLE_SERVO);
    }

    @Override
    public void loop() {
        servo.setPosition(gamepad1.right_stick_y);
        telemetry.addData("servo pos: ", gamepad1.right_stick_y);
    }
}
