package org.firstinspires.ftc.teamcode.subsystems.vision;

import static org.firstinspires.ftc.teamcode.subsystems.vision.VisionConfiguration.MAX_REL_Y;

import org.firstinspires.ftc.teamcode.helpers.enums.Alliance;
import org.firstinspires.ftc.teamcode.subsystems.vision.OrientationDeterminerPostProcessor.SampleOrientation;

import java.util.List;
import java.util.Objects;

public class BestSampleDeterminer {
    /**
     * Determine the most efficient sample to pick up based on the samples provided.
     * <p>
     * For now, the most efficient sample one that is:
     * 1. Within the legal extension limit
     * <p>
     * 2. With the lowest coef as calculated by the formula: 2 * x + y.
     * <p>
     * What I wanted to achieve with this formula is to prioritize samples that are the fastest
     * to pick up. Moving side to side is inevitably going to take longer than moving the slides
     * up or down, so I decided to double the distance left/right to have a more significant
     * effect on the final result. This way, the robot will prioritize samples that are closer.
     * <p>
     * 3. If two samples have the same coef, the one that is yellow will be prioritized over a
     * blue or red one.
     *
     * @param samples The samples to choose from.
     * @return The best sample to pick up.
     */
    public static SampleOrientation determineBestSample(List<SampleOrientation> samples, Alliance alliance) {
        SampleOrientation bestSample = null;
        double bestCoef = Double.MAX_VALUE;

        for (SampleOrientation sample : samples) {
            if (sample == null) continue;
            if (sample.relativeY > MAX_REL_Y) continue;

            if (!sample.color.equals("yellow") && !sample.color.equals(alliance.name)) continue;

            double coef = 2 * Math.abs(sample.relativeX) + Math.abs(sample.relativeY);

            if (coef < bestCoef || (coef == bestCoef && Objects.equals(sample.color, "yellow"))) {
                bestSample = sample;
                bestCoef = coef;
            }
        }

        return bestSample;
    }
}
