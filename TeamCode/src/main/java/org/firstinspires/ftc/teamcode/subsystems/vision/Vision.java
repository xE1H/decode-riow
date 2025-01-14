package org.firstinspires.ftc.teamcode.subsystems.vision;

import static org.firstinspires.ftc.teamcode.subsystems.vision.VisionConfiguration.CAMERA_NAME;
import static org.firstinspires.ftc.teamcode.subsystems.vision.VisionConfiguration.RESOLUTION;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.vision.VisionPortal;

public class Vision extends VLRSubsystem<Vision> {
    private VisionPortal portal;

    @Override
    protected void initialize(HardwareMap hardwareMap) {
        VisionPortal.Builder builder = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, CAMERA_NAME))
                .setCameraResolution(RESOLUTION)
                .enableLiveView(true)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(new SampleProcessor());

        portal = builder.build();
        portal.resumeStreaming();
    }
}
