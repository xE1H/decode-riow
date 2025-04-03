package org.firstinspires.ftc.teamcode.helpers.utils.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.helpers.opmode.VLRLinearOpMode;
import org.firstinspires.ftc.teamcode.subsystems.limelight.LimelightYoloReader;

@TeleOp(group = "Utils")
public class TestYoloReader extends VLRLinearOpMode {
    @Override
    public void run() {
        LimelightYoloReader reader = new LimelightYoloReader();
        waitForStart();
        while (opModeIsActive()) {
            System.out.println(reader.getBestSampleWithRetry());
            sleep(2000);
        }
    }
}
