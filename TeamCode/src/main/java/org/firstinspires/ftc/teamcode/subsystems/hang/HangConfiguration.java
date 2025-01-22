package org.firstinspires.ftc.teamcode.subsystems.hang;

import com.acmerobotics.dashboard.config.Config;

@Config
public interface HangConfiguration {
    String LEFT_AXON = "leftHang";
    String RIGHT_AXON = "rightHang";

    enum TargetPosition {
        DOWN (0),
        HALF(0.75),
        UP (1);
        public final double pos;
        TargetPosition(double pos) {this.pos = pos;}
    }
}