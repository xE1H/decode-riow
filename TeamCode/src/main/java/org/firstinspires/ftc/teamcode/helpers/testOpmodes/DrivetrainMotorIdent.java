package org.firstinspires.ftc.teamcode.helpers.testOpmodes;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.helpers.opmode.VLRLinearOpMode;
import org.firstinspires.ftc.teamcode.subsystems.chassis.ChassisConfiguration;

@Disabled
@TeleOp(name = "Drivetrain Motor Identification", group = "Utils")
public class DrivetrainMotorIdent extends VLRLinearOpMode {
    @Override
    public void run() {
        MotorEx lf = new MotorEx(hardwareMap, ChassisConfiguration.MOTOR_LEFT_FRONT);
        MotorEx rf = new MotorEx(hardwareMap, ChassisConfiguration.MOTOR_RIGHT_FRONT);
        MotorEx lb = new MotorEx(hardwareMap, ChassisConfiguration.MOTOR_LEFT_BACK);
        MotorEx rb = new MotorEx(hardwareMap, ChassisConfiguration.MOTOR_RIGHT_BACK);

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
