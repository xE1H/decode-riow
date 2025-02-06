package org.firstinspires.ftc.teamcode.subsystems.vision;

import static org.firstinspires.ftc.teamcode.subsystems.vision.VisionConfiguration.*;

import android.graphics.Canvas;

import org.firstinspires.ftc.teamcode.subsystems.vision.recog.YoloV11Inference;
import org.firstinspires.ftc.teamcode.subsystems.vision.recog.YoloV11VisionPostProcessor;
import org.firstinspires.ftc.teamcode.subsystems.vision.utils.Point3d;
import org.firstinspires.ftc.teamcode.subsystems.vision.utils.RayGroundIntersectionProcessor;
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

        public String color;


        public SampleOrientation(double relativeX, double relativeY, double relativeZ, boolean isVerticallyOriented, String label) {
            this.relativeX = relativeX;
            this.relativeY = relativeY;
            this.relativeZ = relativeZ;
            this.isVerticallyOriented = isVerticallyOriented;

            this.color = label.replace("sample", "");
        }
    }

    public OrientationDeterminerPostProcessor() {
        this.rgip = new RayGroundIntersectionProcessor(FX, FY, CX, CY, POS_X, POS_Y, POS_Z, LEFT_ANGLE, DOWN_ANGLE);
    }

    @Override
    public void processDetections(Mat undistorted, List<YoloV11Inference.Detection> detectionList) {
        samples.clear();
        for (YoloV11Inference.Detection detection : detectionList) {
            // Have to turn them back since the model was not trained on rotated images
            int[] point1 = turnBackPoint((int) detection.x1, (int) detection.y1);
            int[] point2 = turnBackPoint((int) detection.x2, (int) detection.y2);

            int x1 = point1[0];
            int y1 = point1[1];

            int x2 = point2[0];
            int y2 = point2[1];

            int width = Math.abs(x2 - x1);
            int height = Math.abs(y2 - y1);

            // Very simple check for orientation. Is bound to fail if the sample is not near the
            // center of the image, but will probably work well enough for our use case.
            boolean isVerticallyOriented = width > height;
            // rgip x negative when left
            Point3d coords = rgip.getWorldCoordinates(RESOLUTION.getWidth() - ((x1 + x2) / 2.0), (y1 + y2) / 2.0);
            System.out.println("Corrected pixel coordinates: x: " +  ((x1 + x2) / 2.0) + " y: " + ((y1 + y2) / 2.0));

            samples.add(new SampleOrientation(coords.x, coords.y, coords.z, isVerticallyOriented, detection.label));
        }
        for (SampleOrientation sample : samples) {
            System.out.println("Sample: " + sample.color + " at " + sample.relativeX + ", " + sample.relativeY + ", " + sample.relativeZ + " is " + (sample.isVerticallyOriented ? "vertically" : "horizontally") + " oriented");
        }
    }

    public List<SampleOrientation> getSamples() {
        return samples;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
    }


    private int[] turnBackPoint(int x, int y) {
        return new int[]{RESOLUTION.getWidth() - y, x};
    }
}
