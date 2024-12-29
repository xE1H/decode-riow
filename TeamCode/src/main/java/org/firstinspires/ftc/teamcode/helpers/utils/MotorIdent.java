package org.firstinspires.ftc.teamcode.helpers.utils;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.helpers.opmode.VLRLinearOpMode;

@TeleOp(name = "MotorIdent")
public class MotorIdent extends VLRLinearOpMode {
    @Override
    public void run() {
        MotorEx lf = new MotorEx(hardwareMap, "MotorLeftFront");
        MotorEx rf = new MotorEx(hardwareMap, "MotorRightFront");
        MotorEx lb = new MotorEx(hardwareMap, "MotorLeftBack");
        MotorEx rb = new MotorEx(hardwareMap, "MotorRightBack");

        MotorEx motors[] = {lf, rf, lb, rb};
        String motornames[] = {"lf", "rf", "lb", "rb"};
        while (true) {
            for (int i = 0; i < 4; i++) {
                motors[i].set(1);
                telemetry.addData("Motor", motornames[i]);
                telemetry.update();
                sleep(10000);
                motors[i].set(0);
            }
        }
    }
}
