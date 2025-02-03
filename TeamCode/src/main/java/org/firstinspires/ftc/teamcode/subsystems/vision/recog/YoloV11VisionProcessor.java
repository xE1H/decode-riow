package org.firstinspires.ftc.teamcode.subsystems.vision.recog;

import static org.firstinspires.ftc.teamcode.subsystems.vision.VisionConfiguration.*;
import static org.firstinspires.ftc.teamcode.subsystems.vision.recog.YoloV11VisionProcessorConfig.LABELS;
import static org.firstinspires.ftc.teamcode.subsystems.vision.recog.YoloV11VisionProcessorConfig.LABEL_COLORS;
import static org.firstinspires.ftc.teamcode.subsystems.vision.recog.YoloV11VisionProcessorConfig.CONFIDENCE_THRESHOLD;
import static org.firstinspires.ftc.teamcode.subsystems.vision.recog.YoloV11VisionProcessorConfig.MODEL_FILE_PATH;
import static org.firstinspires.ftc.teamcode.subsystems.vision.recog.YoloV11VisionProcessorConfig.MODEL_INPUT_SIZE;
import static org.firstinspires.ftc.teamcode.subsystems.vision.recog.YoloV11VisionProcessorConfig.X_OFFSET;
import static org.firstinspires.ftc.teamcode.subsystems.vision.recog.YoloV11VisionProcessorConfig.Y_OFFSET;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class YoloV11VisionProcessor implements VisionProcessor {
    private List<YoloV11Inference.Detection> detectionList = new ArrayList<>();
    private YoloV11Inference detector;

    private boolean enabled = false;
    private boolean frameProcessed;
    private YoloV11VisionPostProcessor postProcessor;

    Mat cameraMatrix = new Mat(3, 3, CvType.CV_64F);
    Mat distCoeffs = new Mat(1, 5, CvType.CV_64F);

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        detector = new YoloV11Inference(MODEL_FILE_PATH, MODEL_INPUT_SIZE, CONFIDENCE_THRESHOLD, LABELS, Math.max(width, height), Y_OFFSET, X_OFFSET);

        double[] calibrationData = new double[]{
                FX, 0, CX,
                0, FY, CY,
                0, 0, 1
        };
        cameraMatrix.put(0, 0, calibrationData);

        distCoeffs = new Mat(1, 5, CvType.CV_64F);
        double[] distCoeffData = new double[]{K1, K2, P1, P2, K3};
        distCoeffs.put(0, 0, distCoeffData);
    }

    public void setPostProcessor(YoloV11VisionPostProcessor postProcessor) {
        this.postProcessor = postProcessor;
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        if (!enabled) {
            frameProcessed = false;
            return null;
        }

        Mat undistorted = new Mat();
        Calib3d.undistort(frame, undistorted, cameraMatrix, distCoeffs);

        Mat rotated = new Mat();
        Core.rotate(undistorted, rotated, Core.ROTATE_90_COUNTERCLOCKWISE);


        Bitmap bitmap = getBitmap(rotated);

        detectionList = detector.detect(bitmap);

        for (YoloV11Inference.Detection detection : detectionList) {
            System.out.println("Detected: " + detection.label + " at " + detection.x1 + ", " + detection.y1 + " to " + detection.x2 + ", " + detection.y2);
        }

        if (postProcessor != null) {
            postProcessor.processDetections(undistorted, detectionList);
        }

        frameProcessed = true;
        return null;
    }

    /**
     * @noinspection SuspiciousNameCombination
     */
    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        for (YoloV11Inference.Detection detection : detectionList) {
            Paint paint = new Paint();
            paint.setColor(LABEL_COLORS.get(detection.label));
            paint.setStyle(Paint.Style.STROKE);
            paint.setStrokeWidth(5);

            float newX1 = RESOLUTION.getWidth() - detection.y1;
            float newY1 = detection.x1;
            float newX2 = RESOLUTION.getWidth() - detection.y2;
            float newY2 = detection.x2;

            canvas.drawRect(
                    newX1 * scaleBmpPxToCanvasPx,
                    newY1 * scaleBmpPxToCanvasPx,
                    newX2 * scaleBmpPxToCanvasPx,
                    newY2 * scaleBmpPxToCanvasPx,
                    paint
            );
        }
        if (postProcessor != null) {
            postProcessor.onDrawFrame(canvas, onscreenWidth, onscreenHeight, scaleBmpPxToCanvasPx, scaleCanvasDensity, userContext);
        }
    }

    private Bitmap getBitmap(Mat frame) {
        Bitmap bitmap = null;
        if (frame == null) {
            return null;
        }

        try {
            Mat convertedMat = new Mat();

            // If the Mat is in BGR format, convert it to RGBA
            if (frame.channels() == 3) {
                Imgproc.cvtColor(frame, convertedMat, Imgproc.COLOR_BGR2RGBA);
            } else if (frame.channels() == 1) {
                Imgproc.cvtColor(frame, convertedMat, Imgproc.COLOR_GRAY2RGBA);
            } else {
                convertedMat = frame;
            }

            bitmap = Bitmap.createBitmap(convertedMat.cols(), convertedMat.rows(),
                    Bitmap.Config.ARGB_8888);

            Utils.matToBitmap(convertedMat, bitmap);

            if (convertedMat != frame) {
                convertedMat.release();
            }

        } catch (Exception e) {
            e.printStackTrace();
            if (bitmap != null) {
                bitmap.recycle();
                bitmap = null;
            }
        }
        return bitmap;
    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    public boolean isFrameProcessed() {
        return frameProcessed;
    }
}
