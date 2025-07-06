package org.firstinspires.ftc.teamcode.helpers.testOpmodes;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;

import com.ThermalEquilibrium.homeostasis.Filters.FilterAlgorithms.LowPassFilter;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config

public class test implements Runnable{

    private DcMotorEx motor0, motor1, motor2;


    public test(DcMotorEx motor, DcMotorEx motor1, DcMotorEx motor2) {
        this.motor0 = motor;
        this.motor1 = motor1;
        this.motor2 = motor2;
    }


    @Override
    public void run(){
        while (true){
            motor0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor0.setPower(-0.2);
            motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor1.setPower(-0.2);
            motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor2.setPower(-0.2);
            System.out.println("BOMBO BREAKING");
        }
    }
}