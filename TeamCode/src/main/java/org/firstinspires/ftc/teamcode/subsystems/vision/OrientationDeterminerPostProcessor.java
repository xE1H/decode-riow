package org.firstinspires.ftc.teamcode.subsystems.vision;

import static org.firstinspires.ftc.teamcode.subsystems.vision.VisionConfiguration.*;

import android.graphics.Canvas;

import org.opencv.core.Mat;

import java.util.ArrayList;
import java.util.List;

public class OrientationDeterminerPostProcessor extends YoloV11VisionPostProcessor {
    List<SampleOrientation> samples = new ArrayList<>();

    private RayGroundIntersectionProcessor rgip;

    public class SampleOrientation {
        public double relativeX;
        public double relativeY;
        public double relativeZ;

        public boolean isVerticallyOriented;

        public SampleOrientation(double relativeX, double relativeY, double relativeZ, boolean isVerticallyOriented) {
            this.relativeX = relativeX;
            this.relativeY = relativeY;
            this.relativeZ = relativeZ;
            this.isVerticallyOriented = isVerticallyOriented;
        }
    }

    public OrientationDeterminerPostProcessor() {
        this.rgip = new RayGroundIntersectionProcessor(FX, FY, CX, CY, POS_X, POS_Y, POS_Z, LEFT_ANGLE, DOWN_ANGLE);
    }

    @Override
    public void processDetections(Mat undistorted, List<YoloV11Inference.Detection> detectionList) {
        for (YoloV11Inference.Detection detection : detectionList) {
            int[] point1 = turnBackPoint((int) detection.x1, (int) detection.y1);
            int[] point2 = turnBackPoint((int) detection.x2, (int) detection.y2);

            int x1 = point1[0];
            int y1 = point1[1];

            int x2 = point2[0];
            int y2 = point2[1];

            int width = x2 - x1;
            int height = y2 - y1;
            boolean isVerticallyOriented = width < height;

            Point3d coords = rgip.getWorldCoordinates(x1 + width / 2.0, y1 + height / 2.0);

            samples.add(new SampleOrientation(coords.x, coords.y, coords.z, isVerticallyOriented));
        }
    }

    public List<SampleOrientation> getSamples() {
        return samples;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
    }


    private int[] turnBackPoint(int x, int y) {
        return new int[]{y, RESOLUTION.getWidth() - x};
    }
}
