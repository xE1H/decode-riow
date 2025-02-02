package org.firstinspires.ftc.teamcode.subsystems.vision;

import static org.firstinspires.ftc.teamcode.subsystems.vision.VisionConfiguration.CAMERA_NAME;
import static org.firstinspires.ftc.teamcode.subsystems.vision.VisionConfiguration.RESOLUTION;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.vision.recog.YoloV11VisionPostProcessor;
import org.firstinspires.ftc.teamcode.subsystems.vision.recog.YoloV11VisionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.List;

public class Vision extends VLRSubsystem<Vision> {
    private final YoloV11VisionProcessor processor = new YoloV11VisionProcessor();

    private final OrientationDeterminerPostProcessor postProcessor = new OrientationDeterminerPostProcessor();

    @Override
    protected void initialize(HardwareMap hardwareMap) {
        VisionPortal.Builder builder = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, CAMERA_NAME))
                .setCameraResolution(RESOLUTION)
                .enableLiveView(true)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(processor);

        processor.setPostProcessor(postProcessor);

        VisionPortal portal = builder.build();
        portal.resumeStreaming();
    }

    public List<OrientationDeterminerPostProcessor.SampleOrientation> getSampleOrientations() {
        return postProcessor.getSamples();
    }

    public void setEnabled(boolean enabled) {
        processor.setEnabled(enabled);
    }

    public boolean isFrameProcessed() {
        return processor.isFrameProcessed();
    }
}
