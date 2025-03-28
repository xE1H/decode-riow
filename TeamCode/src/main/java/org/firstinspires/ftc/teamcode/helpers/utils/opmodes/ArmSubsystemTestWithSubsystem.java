package org.firstinspires.ftc.teamcode.helpers.utils.opmodes;

import static org.firstinspires.ftc.teamcode.Points_specimen.START_POSE;

import com.acmerobotics.dashboard.config.Config;
import com.outoftheboxrobotics.photoncore.Photon;
import com.pedropathing.follower.Follower;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.helpers.utils.GlobalConfig;
import org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorSubsystem;
import org.firstinspires.ftc.teamcode.helpers.opmode.VLRLinearOpMode;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideSubsystem;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Photon
@Config
@TeleOp()

public class ArmSubsystemTestWithSubsystem extends VLRLinearOpMode {
    ArmRotatorSubsystem rotator;
    ArmSlideSubsystem slides;

    public static ArmRotatorConfiguration.TargetAngle targetAngle = ArmRotatorConfiguration.TargetAngle.RETRACT;
    public static ArmSlideConfiguration.TargetPosition targetPosition = ArmSlideConfiguration.TargetPosition.RETRACTED;
    public static double targetArmAngle = 0;
    private double prevAngle = 0;

    @Override
    public void run() {
        VLRSubsystem.requireSubsystems(ArmRotatorSubsystem.class, ArmSlideSubsystem.class);
        VLRSubsystem.initializeAll(hardwareMap);

        Constants.setConstants(FConstants.class, LConstants.class);
        Follower f = new Follower(hardwareMap);
        f.updatePose();

        rotator = VLRSubsystem.getRotator();
        slides = VLRSubsystem.getSlides();

        GlobalConfig.DEBUG_MODE = true;

        waitForStart();

        while(opModeIsActive()){
            if (gamepad1.triangle){
                rotator.setTargetPosition(120);
            }
            else if (gamepad1.cross){
                rotator.setTargetAngle(ArmRotatorConfiguration.TargetAngle.RETRACT);
            }

            else if (gamepad1.circle){
                slides.setTargetPosition(0.9);
            }

            else if (gamepad1.square){
                slides.setTargetPosition(0.01);
            }
//            if (prevAngle != targetArmAngle){
//                armSubsystem.setTargetPosition(targetArmAngle);
//                prevAngle = targetArmAngle;
//            }
            //armSubsystem.setTargetAngle(targetAngle);
            //slides.setTargetPosition(targetPosition);

            telemetry.update();
        }
    }
}