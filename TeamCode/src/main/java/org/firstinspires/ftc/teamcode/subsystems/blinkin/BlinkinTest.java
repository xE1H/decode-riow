package org.firstinspires.ftc.teamcode.subsystems.blinkin;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.helpers.opmode.VLRLinearOpMode;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;

@TeleOp(name = "Blinkin test")
public class BlinkinTest extends VLRLinearOpMode {
    private boolean prevUP = false;
    private boolean prevDown = false;

    @Override
    public void run() {
        VLRSubsystem.requireSubsystems(BlinkinSubsystem.class);
        VLRSubsystem.initializeAll(hardwareMap);

        telemetry.addLine("press DPAD UP and DPAD DOWN to cycle between blinkin patterns");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()){
            if (gamepad1.dpad_up && !prevUP){
                VLRSubsystem.getInstance(BlinkinSubsystem.class).next();
            }

            else if (gamepad1.dpad_down && !prevDown){
                VLRSubsystem.getInstance(BlinkinSubsystem.class).previous();
            }

            prevUP = gamepad1.dpad_up;
            prevDown = gamepad1.dpad_down;

            telemetry.update();
        }
    }
}
