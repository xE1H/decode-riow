package org.firstinspires.ftc.teamcode.subsystems.claw;

import com.acmerobotics.dashboard.config.Config;

@Config
public interface ClawConfiguration {
    String ANGLE_SERVO = "angle";
    String TWIST_SERVO = "twist";
    String GRAB_SERVO = "claw";
    String PROXIMITY_SENSOR = "clawProximity";

    double HORIZONTAL_ROTATION_MIN = 0;
    double HORIZONTAL_ROTATION_MAX = 1;

    double CLAW_PROXIMITY_THRESHOLD = 0.84;

    enum VerticalRotation {
        DOWN(1),
        UP(0.08),
        DEPOSIT(0.35);

        public final double pos;
        VerticalRotation(double pos) {
            this.pos = pos;
        }
    }

    enum HorizontalRotation {
        NORMAL(0.5);

        public final double pos;

        HorizontalRotation(double pos) {
            this.pos = pos;
        }
    }

    enum GripperState {
        CLOSED(1),
        CLOSED_LOOSE(0.9),
        OPEN(0.22);

        public final double pos;

        GripperState(double pos) {
            this.pos = pos;
        }
    }
}