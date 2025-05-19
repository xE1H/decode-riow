package org.firstinspires.ftc.teamcode.helpers.testOpmodes;

import com.acmerobotics.dashboard.config.Config;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ServoCalibration", group = "Utils")
@Photon
@Config
public class ServoCalibration extends LinearOpMode {
    private boolean prevState = false;
    public static int current_i = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        String[] servoNames = {"twist", "angle", "claw"};
        double location = 0;
        waitForStart();
        while (opModeIsActive()) {
            Servo s = hardwareMap.get(Servo.class, servoNames[current_i]);
            telemetry.addData("servo", servoNames[current_i]);
            telemetry.addData("location", location);
            telemetry.update();


            if (gamepad1.a && !prevState) {
                current_i++;
                current_i %= servoNames.length;
                location = 0;
                prevState = gamepad1.a;
            }

            if (gamepad1.dpad_up) {
                location += 0.02;
            } else if (gamepad1.dpad_down) {
                location -= 0.02;
            }
            location = Math.max(0, Math.min(1, location));
            s.setPosition(location);

            sleep(15);
        }
    }
}
