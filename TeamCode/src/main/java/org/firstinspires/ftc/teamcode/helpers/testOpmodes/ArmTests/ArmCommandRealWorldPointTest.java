package org.firstinspires.ftc.teamcode.helpers.testOpmodes.ArmTests;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.helpers.opmode.VLRLinearOpMode;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.helpers.utils.GlobalConfig;
import org.firstinspires.ftc.teamcode.subsystems.arm.MainArmConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.arm.MainArmConfiguration.OFFSET_REFERENCE_PLANE;
import org.firstinspires.ftc.teamcode.subsystems.arm.MainArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.SetArmPosition;

@Config
@Photon
@TeleOp(group = "Utils")

public class ArmCommandRealWorldPointTest extends VLRLinearOpMode {
    public static double xOffset = 0;
    public static double yOffset = 0;

    private double prevX = 0;
    private double prevY = 0;

    @Override
    public void run() {
        VLRSubsystem.requireSubsystems(MainArmSubsystem.class);
        VLRSubsystem.initializeAll(hardwareMap);

        GlobalConfig.DEBUG_MODE = true;

        waitForStart();

        while(opModeIsActive()){
            if (prevX != xOffset || prevY != yOffset){
                CommandScheduler.getInstance().schedule(new SetArmPosition().XY(xOffset, yOffset, OFFSET_REFERENCE_PLANE.FRONT, MainArmConfiguration.GAME_PIECE_TYPE.SAMPLE));
                prevX = xOffset;
                prevY = yOffset;
            }

            telemetry.update();
        }
    }
}