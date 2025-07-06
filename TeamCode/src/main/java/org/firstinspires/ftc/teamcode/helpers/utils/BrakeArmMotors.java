package org.firstinspires.ftc.teamcode.helpers.utils;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleMotor;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleRevHub;

@Config

public class BrakeArmMotors implements Runnable{

    private CuttleMotor rotator, motor0, motor1, motor2;
    private CuttleRevHub expansionHub;


    public BrakeArmMotors(HardwareMap hardwareMap) {
        expansionHub = new CuttleRevHub(hardwareMap, "Expansion Hub 2");
        motor0 = new CuttleMotor(expansionHub, 1);
        motor1 = new CuttleMotor(expansionHub, 2);
        motor2 = new CuttleMotor(expansionHub, 3);
        rotator = new CuttleMotor(expansionHub, 0);
    }


    @Override
    public void run(){
        while (true){
            motor0.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
            //motor0.setPower(-0.2);

            motor1.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
            //motor1.setPower(-0.2);

            motor2.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
            //motor2.setPower(-0.2);

            rotator.setZeroPowerBehaviour(DcMotor.ZeroPowerBehavior.BRAKE);
            //rotator.setPower(-0.2);
            System.out.println("BOMBO BREAKING");
        }
    }
}