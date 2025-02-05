package org.firstinspires.ftc.teamcode.subsystems.hang;

import com.acmerobotics.dashboard.config.Config;

@Config
public interface HangConfiguration {
    String LEFT_AXON = "leftHang";
    String RIGHT_AXON = "rightHang";

    String LEFT_ANALOG = "leftAnalog";
    String RIGHT_ANALOG = "rightAnalog";

    double velocityThreshold = 26;
    double positionDelta = 50;

    enum TargetPosition {
        DOWN,
        UP
    }
}