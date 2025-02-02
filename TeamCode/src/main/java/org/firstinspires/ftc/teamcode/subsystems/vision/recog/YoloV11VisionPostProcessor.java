package org.firstinspires.ftc.teamcode.subsystems.vision.recog;

import android.graphics.Canvas;

import org.opencv.core.Mat;

import java.util.List;

public abstract class YoloV11VisionPostProcessor {
    public abstract void processDetections(Mat undistorted, List<YoloV11Inference.Detection> detectionList);

    public abstract void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext);
}
