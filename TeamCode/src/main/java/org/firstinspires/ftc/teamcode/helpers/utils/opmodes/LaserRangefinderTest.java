package org.firstinspires.ftc.teamcode.helpers.utils.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@Autonomous
public class LaserRangefinderTest extends LinearOpMode {
    RevTouchSensor pin0;
    AnalogInput pin1;

    @Override
    public void runOpMode(){
        pin0 = hardwareMap.get(RevTouchSensor.class, "digital0");
        //pin1 = hardwareMap.get(AnalogInput.class, "analog1");
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("digital 0", pin0.isPressed());
            System.out.println("DISTANCE: " + pin0.isPressed());
            telemetry.update();
        }
    }

}
