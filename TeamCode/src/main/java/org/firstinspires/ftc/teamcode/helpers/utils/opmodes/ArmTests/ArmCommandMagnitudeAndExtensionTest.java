package org.firstinspires.ftc.teamcode.helpers.utils.opmodes.ArmTests;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.helpers.utils.GlobalConfig;
import org.firstinspires.ftc.teamcode.subsystems.arm.MainArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.SetArmPosition;
import org.firstinspires.ftc.teamcode.helpers.opmode.VLRLinearOpMode;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.chassis.Chassis;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;

@Config
@Photon
@TeleOp()
public class ArmCommandMagnitudeAndExtensionTest extends VLRLinearOpMode {
    public static double extension = 0;
    public static double angle = 0;

    private double prevExtension = 0;
    private double prevAngle = 0;

    @Override
    public void run() {
        VLRSubsystem.requireSubsystems(MainArmSubsystem.class, ClawSubsystem.class, Chassis.class);
        VLRSubsystem.initializeAll(hardwareMap);

        GlobalConfig.DEBUG_MODE = true;

        waitForStart();

        while(opModeIsActive()){
            if (extension != prevExtension){
                prevExtension = extension;
                //VLRSubsystem.getArm().setTargetExtension(extension);
                CommandScheduler.getInstance().schedule(new SetArmPosition().extension(extension));
            }

            if (angle != prevAngle){
                prevAngle = angle;
                CommandScheduler.getInstance().schedule(new SetArmPosition().angleDegrees(angle));
            }

            telemetry.update();
        }
    }
}