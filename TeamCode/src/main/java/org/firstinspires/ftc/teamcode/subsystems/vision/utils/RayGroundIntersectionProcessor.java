package org.firstinspires.ftc.teamcode.subsystems.vision.utils;

import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Core;

public class RayGroundIntersectionProcessor {
    private double fx, fy, cx, cy;
    private double posX, posY, posZ;
    private double leftAngle, downAngle;

    public RayGroundIntersectionProcessor(double fx, double fy, double cx, double cy,
                        double posX, double posY, double posZ,
                        double leftAngle, double downAngle) {
        this.fx = fx;
        this.fy = fy;
        this.cx = cx;
        this.cy = cy;
        this.posX = posX;
        this.posY = posY;
        this.posZ = posZ;
        this.leftAngle = leftAngle;
        this.downAngle = downAngle;
    }

    private Mat createRotationMatrix(double leftAngle, double downAngle) {
        // Convert to radians
        double theta = Math.toRadians(leftAngle);
        double phi = Math.toRadians(-downAngle);

        // Create rotation matrices
        Mat rRot = new Mat(3, 3, CvType.CV_64F);
        rRot.put(0, 0,
                0, 0, -1,
                0, 1, 0,
                1, 0, 0
        );

        Mat rLeft = new Mat(3, 3, CvType.CV_64F);
        rLeft.put(0, 0,
                Math.cos(theta), 0, Math.sin(theta),
                0, 1, 0,
                -Math.sin(theta), 0, Math.cos(theta)
        );

        Mat rDown = new Mat(3, 3, CvType.CV_64F);
        rDown.put(0, 0,
                1, 0, 0,
                0, Math.cos(phi), -Math.sin(phi),
                0, Math.sin(phi), Math.cos(phi)
        );

        // Multiply matrices
        Mat temp = new Mat();
        Mat result = new Mat();
        Core.gemm(rRot, rDown, 1, new Mat(), 0, temp);
        Core.gemm(temp, rLeft, 1, new Mat(), 0, result);

        temp.release();
        rRot.release();
        rLeft.release();
        rDown.release();

        return result;
    }

    public Point3d getWorldCoordinates(double pixelX, double pixelY) {
        // Convert to normalized image coordinates
        double normalizedX = (pixelX - cx) / fx;
        double normalizedY = (pixelY - cy) / fy;

        // Create ray vector
        Mat ray = new Mat(3, 1, CvType.CV_64F);
        ray.put(0, 0, normalizedX, normalizedY, 1.0);

        // Get rotation matrix
        Mat R = createRotationMatrix(leftAngle, downAngle);

        // Transform ray to world coordinates
        Mat worldRay = new Mat();
        Core.gemm(R, ray, 1, new Mat(), 0, worldRay);

        // Normalize worldRay
        double norm = Math.sqrt(
                Math.pow(worldRay.get(0,0)[0], 2) +
                        Math.pow(worldRay.get(1,0)[0], 2) +
                        Math.pow(worldRay.get(2,0)[0], 2)
        );

        double wx = worldRay.get(0,0)[0] / norm;
        double wy = worldRay.get(1,0)[0] / norm;
        double wz = worldRay.get(2,0)[0] / norm;

        // Find intersection with Z=0 plane
        double t = -posZ / wz;
        Point3d intersection = new Point3d(
                posX + t * wx,
                posY + t * wy,
                0.0
        );

        // Clean up
        ray.release();
        R.release();
        worldRay.release();

        return intersection;
    }

    // Convenience method that handles the x-coordinate flip
    public Point3d pixelToWorld(double pixelX, double pixelY) {
        return getWorldCoordinates(1280 - pixelX, pixelY);
    }
}