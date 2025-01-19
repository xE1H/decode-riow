package org.firstinspires.ftc.teamcode.testOpModes;

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
@TeleOp(name = "ArmMotionProfileTuningWSubsystem")

public class ArmSubsystemTest extends VLRLinearOpMode {
    ArmRotatorSubsystem armSubsystem;
    public static ArmRotatorConfiguration.TargetAngle targetAngle = ArmRotatorConfiguration.TargetAngle.RETRACT;
    public static ArmSlideConfiguration.TargetPosition targetPosition = ArmSlideConfiguration.TargetPosition.RETRACTED;

    @Override
    public void run() {
        VLRSubsystem.requireSubsystems(ArmRotatorSubsystem.class, ArmSlideSubsystem.class);
        VLRSubsystem.initializeAll(hardwareMap);

        armSubsystem = VLRSubsystem.getInstance(ArmRotatorSubsystem.class);

        GlobalConfig.DEBUG_MODE = true;
        GlobalConfig.setIsInvertedMotors(false);

        waitForStart();

        while(opModeIsActive()){
            armSubsystem.setTargetAngle(targetAngle);
            armSubsystem.setTargetPosition(targetPosition);
        }
    }
}