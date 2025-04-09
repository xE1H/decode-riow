package org.firstinspires.ftc.teamcode.helpers.utils.opmodes;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.helpers.utils.GlobalConfig;
import org.firstinspires.ftc.teamcode.subsystems.arm.MainArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.SetArmPosition;
import org.firstinspires.ftc.teamcode.helpers.opmode.VLRLinearOpMode;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;


@Photon
@TeleOp()

public class ArmSubsystemTestWithSubsystem extends VLRLinearOpMode {

    @Override
    public void run() {
        VLRSubsystem.requireSubsystems(MainArmSubsystem.class);
        VLRSubsystem.initializeAll(hardwareMap);

        GlobalConfig.DEBUG_MODE = true;

        waitForStart();

        while(opModeIsActive()){
            if (gamepad1.triangle){
                CommandScheduler.getInstance().schedule(new SetArmPosition().angleDegrees(120));
            }
            else if (gamepad1.cross){
                CommandScheduler.getInstance().schedule(new SetArmPosition().angleDegrees(0));
            }

            else if (gamepad1.circle){
                CommandScheduler.getInstance().schedule(new SetArmPosition().extension(0.5));
            }

            else if (gamepad1.square){
                CommandScheduler.getInstance().schedule(new SetArmPosition().extension(0));
            }

            telemetry.update();
        }
    }
}