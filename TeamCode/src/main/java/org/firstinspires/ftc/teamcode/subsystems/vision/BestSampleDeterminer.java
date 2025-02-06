package org.firstinspires.ftc.teamcode.subsystems.vision;

import static org.firstinspires.ftc.teamcode.subsystems.vision.VisionConfiguration.MAX_REL_Y;
import static org.firstinspires.ftc.teamcode.subsystems.vision.VisionConfiguration.MIN_REL_Y;

import org.firstinspires.ftc.teamcode.helpers.enums.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.vision.OrientationDeterminerPostProcessor.SampleOrientation;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

public class BestSampleDeterminer {
    // Proximity parameters - adjust these values based on testing
    private static final double PROXIMITY_THRESHOLD = 4.0; // Distance threshold to consider samples as close
    private static final double PROXIMITY_WEIGHT = 1.5;     // Weight for each nearby sample's impact on the coefficient

    public static SampleOrientation determineBestSample(List<SampleOrientation> samples, Alliance alliance) {
        List<SampleOrientation> validSamples = new ArrayList<>();

        System.out.println("Bing bang " + samples.size());

        // Filter valid samples based on Y bounds and color
        for (SampleOrientation sample : samples) {
            if (sample == null) {
                continue;
            }

            System.out.println("Checking sample " + sample.color);
            System.out.println("X: " + sample.relativeX);
            System.out.println("Y: " + sample.relativeY);

            if (sample.relativeY > MAX_REL_Y || sample.relativeY < MIN_REL_Y) {
                System.out.println("Sample filtered due to Y position");
                continue;
            }

            if (!sample.color.equals("yellow") && !sample.color.equals(alliance.name)) {
                System.out.println("Sample filtered due to color");
                continue;
            }

            validSamples.add(sample);
            System.out.println("Sample added as valid");
        }

        if (validSamples.isEmpty()) {
            return null;
        }

        SampleOrientation bestSample = null;
        double bestCoef = Double.MAX_VALUE;

        // Evaluate each valid sample considering proximity to others
        for (SampleOrientation sample : validSamples) {
            double originalCoef = 2 * Math.abs(sample.relativeX) + Math.abs(sample.relativeY);
            int nearbySamplesCount = 0;

            // Count how many other valid samples are within the proximity threshold
            for (SampleOrientation other : validSamples) {
                if (other == sample) {
                    continue;
                }
                double dx = sample.relativeX - other.relativeX;
                double dy = sample.relativeY - other.relativeY;
                double distance = 2 * Math.abs(dx) + Math.abs(dy);

                if (distance <= PROXIMITY_THRESHOLD) {
                    nearbySamplesCount++;
                }
            }

            // Adjust coefficient by adding penalty based on nearby samples
            double adjustedCoef = originalCoef + (nearbySamplesCount * PROXIMITY_WEIGHT);

            System.out.println("Sample " + sample.color + " original coef: " + originalCoef +
                    ", nearby: " + nearbySamplesCount + ", adjusted coef: " + adjustedCoef);

            // Select the sample with the lowest adjusted coefficient, preferring yellow on ties
            if (adjustedCoef < bestCoef || (adjustedCoef == bestCoef && Objects.equals(sample.color, "yellow"))) {
                bestSample = sample;
                bestCoef = adjustedCoef;
            }
        }

        return bestSample;
    }
}