package org.firstinspires.ftc.teamcode.subsystems.vision;

import static org.firstinspires.ftc.teamcode.subsystems.vision.VisionConfiguration.CAMERA_NAME;
import static org.firstinspires.ftc.teamcode.subsystems.vision.VisionConfiguration.RESOLUTION;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.List;

public class Vision extends VLRSubsystem<Vision> {
    private VisionPortal portal;
    private YoloV11VisionProcessor processor = new YoloV11VisionProcessor();

    private YoloV11VisionPostProcessor postProcessor = new OrientationDeterminerPostProcessor();

    @Override
    protected void initialize(HardwareMap hardwareMap) {
        VisionPortal.Builder builder = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, CAMERA_NAME))
                .setCameraResolution(RESOLUTION)
                .enableLiveView(true)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(processor);

        processor.setPostProcessor(postProcessor);

        portal = builder.build();
        portal.resumeStreaming();
    }
}
