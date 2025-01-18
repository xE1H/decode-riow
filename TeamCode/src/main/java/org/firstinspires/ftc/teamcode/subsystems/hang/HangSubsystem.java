package org.firstinspires.ftc.teamcode.subsystems.hang;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;

@Config
public class HangSubsystem extends VLRSubsystem<HangSubsystem> implements HangConfiguration {
    private Servo left, right;

    @Override
    protected void initialize(HardwareMap hardwareMap) {
        left = hardwareMap.get(Servo.class, LEFT_AXON);
        right = hardwareMap.get(Servo.class, RIGHT_AXON);
        left.setPosition(TargetPosition.DOWN.pos);
        right.setPosition(TargetPosition.DOWN.pos);
    }

    public void setTargetPosition(TargetPosition target) {
        left.setPosition(target.pos);
        right.setPosition(target.pos);
    }
}