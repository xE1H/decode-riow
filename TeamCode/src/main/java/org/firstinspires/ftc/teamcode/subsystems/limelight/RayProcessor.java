package org.firstinspires.ftc.teamcode.subsystems.limelight;

/**
 * This is just a simplified version of the RayGroundIntersectionProcessor class
 * from the old vision system. This only has one angle to worry about, so it should be more reliable
 * in theory.
 */
public class RayProcessor {
    private final double fx, fy;  // Focal lengths
    private final double cx, cy;  // Principal point (optical center)

    private final double camX, camY, camZ;  // Camera position in world coordinates
    private final double tiltAngle;  // Tilt angle in radians

    // Precompute sin and cos of the tilt angle
    private final double cosTheta, sinTheta;

    /**
     * Constructor for the position calculator
     *
     * @param fx               Focal length in x direction (pixels)
     * @param fy               Focal length in y direction (pixels)
     * @param cx               Principal point x-coordinate (pixels)
     * @param cy               Principal point y-coordinate (pixels)
     * @param camX             Camera X position (inches)
     * @param camY             Camera Y position (inches)
     * @param camZ             Camera Z position (inches)
     * @param tiltAngleDegrees Camera tilt angle in degrees (positive = tilted down)
     */
    public RayProcessor(double fx, double fy, double cx, double cy,
                        double camX, double camY, double camZ, double tiltAngleDegrees) {
        this.fx = fx;
        this.fy = fy;
        this.cx = cx;
        this.cy = cy;
        this.camX = camX;
        this.camY = camY;
        this.camZ = camZ;
        this.tiltAngle = Math.toRadians(tiltAngleDegrees);
        this.cosTheta = Math.cos(this.tiltAngle);
        this.sinTheta = Math.sin(this.tiltAngle);
    }

    /**
     * Calculate world X,Y coordinates for an object at a given pixel position and known Z height
     *
     * @param pixelX  X-coordinate in the image
     * @param pixelY  Y-coordinate in the image
     * @param objectZ Z-coordinate in world space (inches from ground)
     * @return double array containing [worldX, worldY] coordinates in inches
     */
    public double[] calculateWorldCoordinates(double pixelX, double pixelY, double objectZ) {
        // Step 1: Convert pixel coordinates to normalized image coordinates
        double normalizedX = (pixelX - cx) / fx;
        double normalizedY = (pixelY - cy) / fy;

        // Step 2: Get ray direction in camera coordinates
        // In camera coordinates, the ray direction is [normalizedX, normalizedY, 1]
        double rayDirX = normalizedX;
        double rayDirY = normalizedY;
        double rayDirZ = 1.0;

        // Step 3: Transform ray direction to world coordinates
        // Rotation matrix for camera tilted down around X-axis
        double worldDirX = rayDirX;
        double worldDirY = cosTheta * rayDirY + sinTheta * rayDirZ;
        double worldDirZ = -sinTheta * rayDirY + cosTheta * rayDirZ;

        // Step 4: Find intersection with Z = objectZ plane
        // We need to find λ where: objectZ = camZ + λ * worldDirZ
        double lambda = (objectZ - camZ) / worldDirZ;

        // Step 5: Calculate world X and Y coordinates
        double worldX = camX + lambda * worldDirX;
        double worldY = camY + lambda * worldDirY;

        return new double[]{worldX, worldY};
    }
}