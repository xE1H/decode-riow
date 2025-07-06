package org.firstinspires.ftc.teamcode.helpers.testOpmodes;

import static org.firstinspires.ftc.teamcode.subsystems.hang.HangConfiguration.LEFT_AXON;
import static org.firstinspires.ftc.teamcode.subsystems.hang.HangConfiguration.RIGHT_AXON;
import static java.lang.Thread.sleep;

import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleMotor;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleRevHub;

import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.hang.HangSubsystem;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;




@Photon
@TeleOp(group = "Utils")
public class TestBreakAfterOpmodeEnd extends LinearOpMode {
    CRServo left, right;


    @Override
    public void runOpMode() {
        DcMotorEx slide0, slide1, slide2;
//        ExecutorService es = Executors.newCachedThreadPool();
//
//        DcMotorStore.testuojam0 = hardwareMap.get(DcMotorEx.class, ArmSlideConfiguration.MOTOR_NAME_0);
//        DcMotorStore.testuojam1 = hardwareMap.get(DcMotorEx.class, ArmSlideConfiguration.MOTOR_NAME_1);
//        DcMotorStore.testuojam2 = hardwareMap.get(DcMotorEx.class, ArmSlideConfiguration.MOTOR_NAME_2);
//
        slide0 = hardwareMap.get(DcMotorEx.class, ArmSlideConfiguration.MOTOR_NAME_0);
        slide1 = hardwareMap.get(DcMotorEx.class, ArmSlideConfiguration.MOTOR_NAME_1);
        slide2 = hardwareMap.get(DcMotorEx.class, ArmSlideConfiguration.MOTOR_NAME_2);
//        es.submit(new NelegalusAfterOpmodeRunnable());
//
//        DcMotorStore.testuojam0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        DcMotorStore.testuojam1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        DcMotorStore.testuojam2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        test test = new test(slide0, slide1, slide2);
        Thread brake = new Thread(test);


        left = hardwareMap.get(CRServo.class, LEFT_AXON);
        right = hardwareMap.get(CRServo.class, RIGHT_AXON);

        new Thread(() -> {
            while (!isStopRequested()) {
                // Background tasks
            }
            CuttleMotor motor = new CuttleMotor(new CuttleRevHub(hardwareMap, CuttleRevHub.HubTypes.CONTROL_HUB), 0);
            ElapsedTime timer = new ElapsedTime();

            while (timer.milliseconds() < 800) {
                System.out.println("bombombinin guzini");
                motor.setPower(0.5);
                sleep(500);
            }
        }).start();

        waitForStart();

        while ((opModeIsActive())){
            sleep(2);
        }
    }

//    @Override
//    public void loop(){
//            System.out.println("BOMBOCLIAT");
//            left.setPower(0);
//            right.setPower(0);
//    }
//
//    @Override
//    public void stop() {
//        ElapsedTime timer = new ElapsedTime();
//        CuttleMotor motor = new CuttleMotor(new CuttleRevHub(hardwareMap, CuttleRevHub.HubTypes.CONTROL_HUB), 0);
//
//        while (timer.milliseconds() < 800) {
//            System.out.println("bombombinin guzini");
//            motor.setPower(0.5);
//            left.setPower(0.15);
//            right.setPower(0.15);
//            try {
//                sleep(1);
//            } catch (InterruptedException e) {
//                throw new RuntimeException(e);
//            }
//        }
//    }

}
