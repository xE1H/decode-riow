package org.firstinspires.ftc.teamcode.helpers.utils.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.helpers.utils.GlobalConfig;
import org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorSubsystem;
import org.firstinspires.ftc.teamcode.helpers.opmode.VLRLinearOpMode;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideSubsystem;

@Photon
@Config
@TeleOp()

public class ArmSubsystemTestWithSubsystem extends VLRLinearOpMode {
    ArmRotatorSubsystem armSubsystem;
    ArmSlideSubsystem slides;

    public static ArmRotatorConfiguration.TargetAngle targetAngle = ArmRotatorConfiguration.TargetAngle.RETRACT;
    public static ArmSlideConfiguration.TargetPosition targetPosition = ArmSlideConfiguration.TargetPosition.RETRACTED;
    public static double targetArmAngle = 0;
    public static double prevAngle = 0;

    @Override
    public void run() {
        VLRSubsystem.requireSubsystems(ArmRotatorSubsystem.class, ArmSlideSubsystem.class);
        VLRSubsystem.initializeAll(hardwareMap);

        armSubsystem = VLRSubsystem.getRotator();
        slides = VLRSubsystem.getSlides();

        GlobalConfig.DEBUG_MODE = true;

        waitForStart();

        while(opModeIsActive()){
            if (prevAngle != targetArmAngle){
                armSubsystem.setTargetPosition(targetArmAngle);
                prevAngle = targetArmAngle;
            }
            //armSubsystem.setTargetAngle(targetAngle);
            slides.setTargetPosition(targetPosition);
        }
    }
}