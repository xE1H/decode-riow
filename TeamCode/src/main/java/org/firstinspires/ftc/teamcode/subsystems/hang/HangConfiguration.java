package org.firstinspires.ftc.teamcode.subsystems.hang;

import com.acmerobotics.dashboard.config.Config;

@Config
public interface HangConfiguration {
    String LEFT_AXON = "leftHang";
    String RIGHT_AXON = "rightHang";

    String LEFT_ANALOG = "leftAnalog";
    String RIGHT_ANALOG = "rightAnalog";

    double leftAnalogThreshold = 238.5;
    double rightAnalogThreshold = 121.5;

    enum TargetPosition {
        DOWN (0),
        HALF(0.75),
        UP (1);
        public final double pos;
        TargetPosition(double pos) {this.pos = pos;}
    }
}