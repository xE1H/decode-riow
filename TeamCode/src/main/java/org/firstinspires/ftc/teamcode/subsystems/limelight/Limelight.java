package org.firstinspires.ftc.teamcode.subsystems.limelight;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.helpers.enums.Alliance;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.jetbrains.annotations.Contract;

import java.util.ArrayList;
import java.util.List;

public class Limelight extends VLRSubsystem<Limelight> {
    private Alliance alliance = Alliance.RED;
    private Limelight3A limelight;
    private RayProcessor rp;

    private List<Sample> cache = new ArrayList<>();

    private static int angleEstIndex = 13;
    private double[] angleEstOutputs = new double[8];

    private boolean enabled = false;

    private long lastUpdate = 0;

    @Override
    protected void initialize(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, LimelightConfiguration.NAME);

        limelight.setPollRateHz(LimelightConfiguration.POLL_RATE_HZ);
        limelight.start();

        limelight.pipelineSwitch(LimelightConfiguration.NOTHING_PIPELINE);

        rp = new RayProcessor(LimelightConfiguration.FX, LimelightConfiguration.FY,
                LimelightConfiguration.CX, LimelightConfiguration.CY,
                LimelightConfiguration.POS_X,
                LimelightConfiguration.POS_Y,
                LimelightConfiguration.POS_Z,
                LimelightConfiguration.TILT_ANGLE);
    }

    @Override
    public void periodic() {
        if (getTimeSinceLastUpdate() > 50) {
            cache = getSampleDetections();
        }
    }

    public List<Sample> getDetections() {
        return cache;
    }

    public void setAlliance(Alliance alliance) {
        this.alliance = alliance;
    }

    public void enable() {
        limelight.pipelineSwitch(alliance == Alliance.BLUE ? LimelightConfiguration.BLUE_PIPELINE : LimelightConfiguration.RED_PIPELINE);
        enabled = true;
    }

    public void disable() {
        limelight.pipelineSwitch(LimelightConfiguration.NOTHING_PIPELINE);
        enabled = false;
    }

    public long getTimeSinceLastUpdate() {
        return System.currentTimeMillis() - lastUpdate;
    }

    private List<Sample> getSampleDetections() {
        if (!enabled) return cache;


        LLResult result = limelight.getLatestResult();
        if (!result.isValid() || result.getStaleness() > LimelightConfiguration.UPDATE_TIMEOUT) {
            System.out.println("ND");
//            System.out.println("No detections because:");
//            System.out.println("Result invalid: " + !result.isValid());
//            System.out.println("Staleness time: " + result.getStaleness());
//            System.out.println("Returning cached value");
            return cache;
        }
        System.out.println("ts pmo");
        angleEstOutputs = result.getPythonOutput();

        List<LLResultTypes.DetectorResult> results = result.getDetectorResults();

        List<Sample> samples = new ArrayList<>();

        for (LLResultTypes.DetectorResult detectorResult : results) {
            System.out.println("R");
            double[] worldCoords = rp.calculateWorldCoordinates(detectorResult.getTargetXPixels(), detectorResult.getTargetYPixels(), 1.5);
            double x = worldCoords[0];
            double y = worldCoords[1];
            double z = 0;
            Sample.Color color = detectorResult.getClassName().equals("yellow") ? Sample.Color.YELLOW :
                    detectorResult.getClassName().equals("blue") ? Sample.Color.BLUE : Sample.Color.RED;
            //Extract two corners of the bounding box


            samples.add(new Sample(x, y, z, getBoundingBoxCorners(detectorResult), color));
        }

        lastUpdate = System.currentTimeMillis();
        System.out.println(samples.size());
        return samples;
    }

    @NonNull
    @Contract("_ -> new")
    private double[] getBoundingBoxCorners(LLResultTypes.DetectorResult detectorResult) {
        //detectorResult.getTargetCorners() returns 4 corners, so we need to find the min and max x and y values
        double x1 = Double.MAX_VALUE;
        double y1 = Double.MAX_VALUE;
        double x2 = Double.MIN_VALUE;
        double y2 = Double.MIN_VALUE;

        for (int i = 0; i < detectorResult.getTargetCorners().size(); i++) {
            double x = detectorResult.getTargetCorners().get(i).get(0);
            double y = detectorResult.getTargetCorners().get(i).get(1);

            x1 = Math.min(x1, x);
            y1 = Math.min(y1, y);
            x2 = Math.max(x2, x);
            y2 = Math.max(y2, y);
        }
        System.out.println("Corners:");
        System.out.println(detectorResult.getTargetCorners().size());
        System.out.println(x1);
        System.out.println(y1);
        System.out.println(x2);
        System.out.println(y2);
        return new double[]{x1, y1, x2, y2};
    }

    public double getAngleEstimation(Sample sample) {
        long startTime = System.currentTimeMillis();

        limelight.pipelineSwitch(LimelightConfiguration.ANGLE_EST_PIPELINE);
        int colorId = sample.color == Sample.Color.YELLOW ? 0 : sample.color == Sample.Color.BLUE ? 1 : 2;
        limelight.updatePythonInputs(new double[]{sample.x1, sample.y1, sample.x2, sample.y2, colorId, angleEstIndex, 0, 0});

        double angle = -360;
        int results = 0;

        while (System.currentTimeMillis() - startTime < 1000) {
            LLResult result = limelight.getLatestResult();
            angleEstOutputs = result.getPythonOutput();
            for (double out : angleEstOutputs) {
                System.out.print(out);
                System.out.print(" ");
            }
            System.out.println();
            if (angleEstOutputs[4] == angleEstIndex) {
                results++;
                if (results < 5) continue;
                // Correct estimation, get coords
                double x1, y1, x2, y2;
                x1 = angleEstOutputs[0];
                y1 = angleEstOutputs[1];
                x2 = angleEstOutputs[2];
                y2 = angleEstOutputs[3];

                double[] point1 = rp.calculateWorldCoordinates(x1, y1, 1.5);
                double[] point2 = rp.calculateWorldCoordinates(x2, y2, 1.5);

                angle = Math.toDegrees(Math.atan2(point2[1] - point1[1], point2[0] - point1[0]));
                break;
            }
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
        angleEstIndex++;
        enable();
        System.out.println("Detected angle: " + angle);
        return angle;
    }

    public static class Sample {
        // Real world coordinates
        public double x, y, z;
        // Camera frame coordinates (bounding box)
        public double x1, y1, x2, y2;
        public Color color;

        public enum Color {
            YELLOW,
            BLUE,
            RED
        }

        public Sample(double x, double y, double z, double[] bounds, Color color) {
            this.x = x;
            this.y = y;
            this.z = z;

            this.x1 = bounds[0];
            this.y1 = bounds[1];
            this.x2 = bounds[2];
            this.y2 = bounds[3];

            this.color = color;
        }
    }
}
