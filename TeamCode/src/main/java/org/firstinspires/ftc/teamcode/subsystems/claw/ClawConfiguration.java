package org.firstinspires.ftc.teamcode.subsystems.claw;

import com.acmerobotics.dashboard.config.Config;

@Config
public interface ClawConfiguration {
    String ANGLE_SERVO = "angle";
    String TWIST_SERVO = "twist";
    String GRAB_SERVO = "claw";
    String ANALOG_ENCODER_LEFT = "analog0";
    String ANALOG_ENCODER_RIGHT = "analog1";

    double angle_down_pos = 0;
    double angle_up_pos = 1;
    double angle_deposit_pos = 0.625;

    double twist_normal_pos = 0.05;
    double twist_flipped_pos = 0.8;

    double TWIST_MIN = 0.04;
    double TWIST_MAX = 0.96;

    double state_closed_normal_pos = 0;
    double state_closed_forced_pos = 0;
    double state_open_pos = 0.8;

    double analog_voltage_left = 0;
    double analog_voltage_right = 0;


    enum TargetAngle {
        DOWN,
        UP,
        DEPOSIT
    }

    enum TargetTwist {
        NORMAL,
        FLIPPED
    }

    enum TargetState {
        CLOSED,
        OPEN
    }
}