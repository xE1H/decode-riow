package org.firstinspires.ftc.teamcode.subsystems.chassis.helpers;

import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;

public class MecanumDriveController extends MecanumDriveWheelSpeeds implements MecanumDriveControllerConfiguration {
    /**
     * Calculations here:
     * https://gm0.org/en/latest/docs/software/concepts/kinematics.html#id3
     */
    public MecanumDriveController(
            double xSpeed, double ySpeed, double zRotation
    ) {
        super(
                xSpeed - ySpeed - (2 * BASE_TRACK_RADIUS * zRotation),
                xSpeed + ySpeed + (2 * BASE_TRACK_RADIUS * zRotation),
                xSpeed + ySpeed - (2 * BASE_TRACK_RADIUS * zRotation),
                xSpeed - ySpeed + (2 * BASE_TRACK_RADIUS * zRotation)
        );
    }
}