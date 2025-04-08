package org.firstinspires.ftc.teamcode.helpers.utils.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@Autonomous
public class LaserRangefinderTest extends LinearOpMode {
    AnalogInput pin0;
    AnalogInput pin1;

    @Override
    public void runOpMode(){
        pin0 = hardwareMap.get(AnalogInput.class, "analog0");
        pin1 = hardwareMap.get(AnalogInput.class, "analog1");

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("digital 0", pin0.getVoltage() / 3.3 * 1000);
            telemetry.addData("digital 1", pin1.getVoltage() / 3.3 * 1000);
            telemetry.update();
        }
    }

}
