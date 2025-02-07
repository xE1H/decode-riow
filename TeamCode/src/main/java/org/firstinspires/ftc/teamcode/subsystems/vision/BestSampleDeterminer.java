package org.firstinspires.ftc.teamcode.subsystems.vision;

import static org.firstinspires.ftc.teamcode.subsystems.vision.VisionConfiguration.MAX_REL_Y;
import static org.firstinspires.ftc.teamcode.subsystems.vision.VisionConfiguration.MIN_REL_Y;

import org.firstinspires.ftc.teamcode.helpers.enums.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.vision.OrientationDeterminerPostProcessor.SampleOrientation;

import java.util.List;
import java.util.Objects;

public class BestSampleDeterminer {
    private static final double X_THRESHOLD = 4; // inches
    private static final double Y_THRESHOLD = 4; // inches
    private static final double NEIGHBOR_PENALTY = 8.0;

    public static SampleOrientation determineBestSample(List<SampleOrientation> samples, Alliance alliance) {
        SampleOrientation bestSample = null;
        double bestCoef = Double.MAX_VALUE;

        System.out.println("Bing bang " + samples.size());

        for (SampleOrientation sample : samples) {
            System.out.println("sempluojam " + sample.color);
            System.out.println("X: " + sample.relativeX);
            System.out.println("Y: " + sample.relativeY);

            if (sample == null) continue;
            if (sample.relativeY > MAX_REL_Y || sample.relativeY < MIN_REL_Y) continue;
            if (!sample.color.equals("yellow") && !sample.color.equals(alliance.name)) continue;

            boolean hasNeighbor = false;
            for (SampleOrientation otherSample : samples) {
                if (otherSample == null || otherSample == sample) continue;
                double dx = Math.abs(sample.relativeX - otherSample.relativeX);
                double dy = Math.abs(sample.relativeY - otherSample.relativeY);
                if (dx <= X_THRESHOLD && dy <= Y_THRESHOLD) {
                    hasNeighbor = true;
                    break;
                }
            }

            double coef = 2 * Math.abs(sample.relativeX) + Math.abs(sample.relativeY);
            if (hasNeighbor) {
                coef += NEIGHBOR_PENALTY;
            }

            if (coef < bestCoef || (coef == bestCoef && Objects.equals(sample.color, "yellow"))) {
                bestSample = sample;
                bestCoef = coef;
            }
        }

        return bestSample;
    }
}
