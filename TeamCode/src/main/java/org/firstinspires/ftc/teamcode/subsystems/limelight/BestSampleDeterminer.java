package org.firstinspires.ftc.teamcode.subsystems.limelight;

import static org.firstinspires.ftc.teamcode.subsystems.vision.VisionConfiguration.MAX_REL_Y;
import static org.firstinspires.ftc.teamcode.subsystems.vision.VisionConfiguration.MIN_REL_Y;

import org.firstinspires.ftc.teamcode.helpers.enums.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.vision.OrientationDeterminerPostProcessor.SampleOrientation;

import java.util.List;
import java.util.Objects;

public class BestSampleDeterminer {
    private static final double X_THRESHOLD = 4; // inches
    private static final double Y_THRESHOLD = 4; // inches
    private static final double NEIGHBOR_PENALTY = 3.0;

    private static final double MAX_REL_X = 7;

    private static final double X_MIN = 50.5;
    private static final double X_MAX = X_MIN + 30;

    public static Limelight.Sample determineBestSample(List<Limelight.Sample> samples, Alliance alliance, double xCoord) {

        Limelight.Sample bestSample = null;
        double bestCoef = Double.MAX_VALUE;

        System.out.println("Bing bang " + samples.size());

        for (Limelight.Sample sample : samples) {
            System.out.println("sempluojam " + sample.color);
            System.out.println("X: " + sample.x);
            System.out.println("Y: " + sample.y);

            if (sample == null) continue;
            if (sample.y > MAX_REL_Y || sample.y < MIN_REL_Y) continue;
            if (Math.abs(sample.x) > MAX_REL_X) continue; // Don't grab samples that might be only half in the frame
            if (xCoord - sample.x > X_MAX || xCoord - sample.x < X_MIN) continue;
            if (!sample.color.equals(Limelight.Sample.Color.YELLOW) && !sample.color.equals(alliance == Alliance.BLUE ? Limelight.Sample.Color.BLUE : Limelight.Sample.Color.RED))
                continue;

            int numberOfNeighbors = 0;
            for (Limelight.Sample otherSample : samples) {
                if (otherSample == null || otherSample == sample) continue;
                double dx = Math.abs(sample.x - otherSample.x);
                double dy = Math.abs(sample.y - otherSample.y);
                if (dx <= X_THRESHOLD && dy <= Y_THRESHOLD) {
                    numberOfNeighbors++;
                }
            }

            double coef = 2 * Math.abs(sample.x) + Math.abs(sample.y);
            coef += NEIGHBOR_PENALTY * numberOfNeighbors;

            if (coef < bestCoef || (coef == bestCoef && sample.color == Limelight.Sample.Color.YELLOW)) {
                bestSample = sample;
                bestCoef = coef;
            }
        }

        return bestSample;
    }
}
