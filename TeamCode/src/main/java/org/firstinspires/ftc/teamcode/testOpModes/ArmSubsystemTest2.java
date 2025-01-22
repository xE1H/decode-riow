package org.firstinspires.ftc.teamcode.testOpModes;

import com.acmerobotics.dashboard.config.Config;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.helpers.opmode.VLRLinearOpMode;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.helpers.utils.GlobalConfig;
import org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideSubsystem;

@Photon
@Config
@TeleOp(name = "ArmMotionProfileTuningWSubsystemAngleDegrees")

public class ArmSubsystemTest2 extends VLRLinearOpMode {
    ArmRotatorSubsystem armSubsystem;
    ArmSlideSubsystem slides;
    public static double angle = 0;
    public static double position = 0;

    @Override
    public void run() {
        VLRSubsystem.requireSubsystems(ArmRotatorSubsystem.class, ArmSlideSubsystem.class);
        VLRSubsystem.initializeAll(hardwareMap);

        armSubsystem = VLRSubsystem.getRotator();
        slides = VLRSubsystem.getSlides();

        GlobalConfig.DEBUG_MODE = true;
        GlobalConfig.setIsInvertedMotors(false);
        GlobalConfig.setIsInvertedArmEncoders(false);
        GlobalConfig.setIsInvertedEncoders(true);

        waitForStart();

        while(opModeIsActive()){
            armSubsystem.setTargetPosition(angle);
            slides.setTargetPosition(position);
        }
    }
}