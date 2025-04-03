package org.firstinspires.ftc.teamcode.subsystems.wiper;

import static org.firstinspires.ftc.teamcode.subsystems.wiper.WiperConfiguration.MAX_POS;
import static org.firstinspires.ftc.teamcode.subsystems.wiper.WiperConfiguration.MIN_POS;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;

public class Wiper extends VLRSubsystem<Wiper> {
    private Servo wiperServo;
    private double lastPos = 0;

    @Override
    protected void initialize(HardwareMap hardwareMap) {
        wiperServo = hardwareMap.get(Servo.class, WiperConfiguration.SERVO_NAME);
        wiperServo.setPosition(MIN_POS);
    }

    public void wipe(double x) {
        if (x == lastPos) return;
        wiperServo.setPosition(MIN_POS - (x * (MIN_POS - MAX_POS)));
        lastPos = x;
    }
}
