package org.firstinspires.ftc.teamcode.subsystems.vision;

import static org.firstinspires.ftc.teamcode.subsystems.vision.VisionConfiguration.*;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Path;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.ConcurrentModificationException;
import java.util.List;

public class SampleProcessor implements VisionProcessor {
    Mat cameraMatrix;
    Mat distCoeffs;

    RayGroundIntersectionProcessor rgip = new RayGroundIntersectionProcessor(FX, FY, CX, CY,
            POS_X, POS_Y, POS_Z - 1.5,
            LEFT_ANGLE, DOWN_ANGLE);

    // Store detection results
    private static class Detection {
        Point center; // pixel coordinates
        Point3d worldPos;
        Rect bounds;
        String color;
        double area;
        MatOfPoint contour; // Store the contour points

        Detection(Point center, Point3d worldPos, Rect bounds, String color, double area, MatOfPoint contour) {
            this.center = center;
            this.worldPos = worldPos;
            this.bounds = bounds;
            this.color = color;
            this.area = area;
            this.contour = contour;
        }
    }

    private List<Detection> latestDetections = new ArrayList<>();
    private Paint boxPaint;
    private Paint textPaint;

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        cameraMatrix = new Mat(3, 3, CvType.CV_64F);
        double[] calibrationData = new double[]{
                FX, 0, CX,
                0, FY, CY,
                0, 0, 1
        };
        cameraMatrix.put(0, 0, calibrationData);

        distCoeffs = new Mat(1, 5, CvType.CV_64F);
        double[] distCoeffData = new double[]{K1, K2, P1, P2, K3};
        distCoeffs.put(0, 0, distCoeffData);

        boxPaint = new Paint();
        boxPaint.setStyle(Paint.Style.STROKE);
        boxPaint.setStrokeWidth(4);

        textPaint = new Paint();
        textPaint.setStyle(Paint.Style.FILL);
        textPaint.setTextSize(24);
        textPaint.setTextAlign(Paint.Align.LEFT);
        textPaint.setAntiAlias(true);
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        latestDetections.clear();

        Mat undistorted = new Mat();
        Calib3d.undistort(frame, undistorted, cameraMatrix, distCoeffs);

        Mat hsv = new Mat();
        Imgproc.cvtColor(undistorted, hsv, Imgproc.COLOR_RGB2HSV);

        String[] colors = {"yellow"};

        for (int i = 0; i < colors.length; i++) {
            Mat mask = new Mat();
            Core.inRange(hsv, LOWER_COLOR_BOUNDS[i], UPPER_COLOR_BOUNDS[i], mask);

            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);

                if (area > MIN_AREA_BOUND && area < MAX_AREA_BOUND) {
                    Rect bounds = Imgproc.boundingRect(contour);
                    Point center = new Point(
                            bounds.x + bounds.width / 2.0,
                            bounds.y + bounds.height / 2.0
                    );

                    Point3d worldPos = rgip.pixelToWorld(center.x, center.y);

                    // Create a copy of the contour for storage
                    MatOfPoint contourCopy = new MatOfPoint();
                    contour.copyTo(contourCopy);

                    latestDetections.add(new Detection(center, worldPos, bounds, colors[i], area, contourCopy));
                }
                contour.release();
            }

            mask.release();
            hierarchy.release();
        }

        hsv.release();
        undistorted.release();

        return latestDetections;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
                            Object userContext) {
        try {
            @SuppressWarnings("unchecked")
            List<Detection> detections = (List<Detection>) userContext;

            if (detections == null) return;

            textPaint.setTextSize(48 * scaleCanvasDensity);

            Paint backgroundPaint = new Paint();
            backgroundPaint.setStyle(Paint.Style.FILL);
            backgroundPaint.setColor(Color.argb(180, 0, 0, 0));

            for (Detection detection : detections) {
                switch (detection.color) {
                    case "red":
                        boxPaint.setColor(Color.RED);
                        textPaint.setColor(Color.RED);
                        break;
                    case "blue":
                        boxPaint.setColor(Color.BLUE);
                        textPaint.setColor(Color.BLUE);
                        break;
                    case "yellow":
                        boxPaint.setColor(Color.YELLOW);
                        textPaint.setColor(Color.YELLOW);
                        break;
                }

                // Draw contour
                Path contourPath = new Path();
                Point[] points = detection.contour.toArray();
                if (points.length > 0) {
                    contourPath.moveTo((float) points[0].x * scaleBmpPxToCanvasPx,
                            (float) points[0].y * scaleBmpPxToCanvasPx);
                    for (int i = 1; i < points.length; i++) {
                        contourPath.lineTo((float) points[i].x * scaleBmpPxToCanvasPx,
                                (float) points[i].y * scaleBmpPxToCanvasPx);
                    }
                    contourPath.close();
                    canvas.drawPath(contourPath, boxPaint);
                }

                // Draw center point
                canvas.drawCircle(
                        (float) detection.center.x * scaleBmpPxToCanvasPx,
                        (float) detection.center.y * scaleBmpPxToCanvasPx,
                        5 * scaleCanvasDensity,
                        boxPaint
                );

                String areaText = String.format("X: %.1f Y: %.1f",
                        detection.center.x, detection.center.y);
                Point3d position = detection.worldPos;
                String posText = String.format("X: %.1f Y: %.1f Z: %.1f", position.x, position.y, position.z);


                float textX = (float) detection.bounds.x * scaleBmpPxToCanvasPx;
                float textY = (float) (detection.bounds.y - 10) * scaleBmpPxToCanvasPx;

                // Measure text bounds
                Paint.FontMetrics fm = textPaint.getFontMetrics();
                float textHeight = fm.bottom - fm.top;
                float areaTextWidth = textPaint.measureText(areaText);
                float posTextWidth = textPaint.measureText(posText);
                float maxWidth = Math.max(areaTextWidth, posTextWidth);

                // Draw background rectangles
                canvas.drawRect(
                        textX - 5,
                        textY - textHeight,
                        textX + maxWidth + 5,
                        textY + 5,
                        backgroundPaint
                );
                canvas.drawRect(
                        textX - 5,
                        textY + 25 * scaleCanvasDensity - textHeight,
                        textX + maxWidth + 5,
                        textY + 25 * scaleCanvasDensity + 5,
                        backgroundPaint
                );

                // Draw text
                canvas.drawText(areaText, textX, textY, textPaint);
                canvas.drawText(posText, textX, textY + 25 * scaleCanvasDensity, textPaint);
            }
        } catch (ConcurrentModificationException e) {
            System.out.println("Vision: Concurrent modification exception caught");
        }
    }
}