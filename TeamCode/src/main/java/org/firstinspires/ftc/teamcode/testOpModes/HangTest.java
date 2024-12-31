package org.firstinspires.ftc.teamcode.testOpModes;

import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.helpers.opmode.VLRLinearOpMode;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.hang.HangConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.hang.HangSubsystem;


@Photon
@TeleOp(name = "HangTest")
public class HangTest extends VLRLinearOpMode {

    @Override
    public void run() {
        VLRSubsystem.requireSubsystems(HangSubsystem.class);
        VLRSubsystem.initializeAll(hardwareMap);

        HangSubsystem hang = VLRSubsystem.getInstance(HangSubsystem.class);
        hang.enable();
        hang.initTelemetry(telemetry);
        waitForStart();


        while (opModeIsActive()) {

            if (gamepad1.left_bumper){
                hang.setTargetPosition(HangConfiguration.TargetPosition.UP);
            }

//            if (gamepad1.right_bumper){
//                hang.setTargetPosition(HangConfiguration.TargetPosition.HANG);
//            }
            if (gamepad1.left_trigger != 0 || gamepad1.right_trigger != 0){
                hang.disable();
                hang.setLeftPower(gamepad1.left_trigger);
                hang.setRightPower(gamepad1.right_trigger);
            }
            else{
                hang.enable();
            }

            telemetry.update();
        }
    }
}