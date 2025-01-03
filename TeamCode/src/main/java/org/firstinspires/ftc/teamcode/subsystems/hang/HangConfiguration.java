package org.firstinspires.ftc.teamcode.subsystems.hang;

import com.acmerobotics.dashboard.config.Config;

@Config
public interface HangConfiguration {
    String LEFT_AXON = "leftHang";
    String RIGHT_AXON = "rightHang";

    String ANALOG_ENCODER_LEFT = "analogHangLeft";
    String ANALOG_ENCODER_RIGHT = "analogHangRight";


    enum TargetPosition {
        DOWN (0),
        UP (-16.5),
        HANG (-10);

        public final double rotations;
        TargetPosition(double rotations) {this.rotations = rotations;}
    }
}