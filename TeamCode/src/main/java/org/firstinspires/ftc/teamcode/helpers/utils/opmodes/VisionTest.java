package org.firstinspires.ftc.teamcode.helpers.utils.opmodes;

import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.helpers.opmode.VLRLinearOpMode;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.vision.Vision;

/** @noinspection ALL*/
@TeleOp(name = "Vision Test", group = "Utils")
@Photon
public class VisionTest extends VLRLinearOpMode {

    @Override
    public void run() {
        VLRSubsystem.requireSubsystems(Vision.class);
        VLRSubsystem.initializeAll(hardwareMap);
        VLRSubsystem.getInstance(Vision.class).setEnabled(true);
        waitForStart();
    }
}
