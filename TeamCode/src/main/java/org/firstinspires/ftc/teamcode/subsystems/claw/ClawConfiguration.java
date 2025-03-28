package org.firstinspires.ftc.teamcode.subsystems.claw;

import com.acmerobotics.dashboard.config.Config;

@Config
public interface ClawConfiguration {
    String ANGLE_SERVO = "angle";
    String TWIST_SERVO = "twist";
    String GRAB_SERVO = "claw";
    String ANALOG_ENCODER_LEFT = "analog0";
    String ANALOG_ENCODER_RIGHT = "analog1";
  
    double HORIZONTAL_ROTATION_MIN = 0.04;
    double HORIZONTAL_ROTATION_MAX = 1;

    double state_closed_normal_pos = 0;
    double state_open_pos = 0.925;

    double analog_voltage_left = 0;
    double analog_voltage_right = 0;

    enum VerticalRotation {
        DOWN(0.95),
        UP(0.02),
        DEPOSIT(0.35);

        public final double pos;
        VerticalRotation(double pos) {
            this.pos = pos;
        }
    }

    enum HorizontalRotation {
        NORMAL(0.5),
        FLIPPED(0.95);

        public final double pos;
        HorizontalRotation(double pos) {
            this.pos = pos;
        }
    }

    enum GripperState {
        CLOSED(0.95),
        OPEN(0.24);

        public final double pos;
        GripperState(double pos) {
            this.pos = pos;
        }
    }
}