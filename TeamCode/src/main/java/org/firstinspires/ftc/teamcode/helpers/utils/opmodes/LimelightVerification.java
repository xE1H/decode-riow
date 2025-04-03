package org.firstinspires.ftc.teamcode.helpers.utils.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.limelight.LimelightYoloReader;

@TeleOp(name="Limelight YOLO output", group = "Utils")
public class LimelightVerification extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        LimelightYoloReader reader = new LimelightYoloReader();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("JSON output", reader.getRawJSONRequest("/detection_boxes.json"));
            telemetry.update();
        }
    }
}
