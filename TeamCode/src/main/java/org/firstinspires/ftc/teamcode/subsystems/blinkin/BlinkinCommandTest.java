package org.firstinspires.ftc.teamcode.subsystems.blinkin;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.helpers.opmode.VLRLinearOpMode;
import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;

@TeleOp(name = "Blinkin command  test")
public class BlinkinCommandTest extends VLRLinearOpMode {
    private boolean prevUP = false;
    private boolean prevDown = false;

    @Override
    public void run() {
        VLRSubsystem.requireSubsystems(BlinkinSubsystem.class);
        VLRSubsystem.initializeAll(hardwareMap);

        waitForStart();

        while (opModeIsActive()){
            if (gamepad1.dpad_up && !prevUP){
                CommandScheduler.getInstance().schedule(new SetPattern().blinkSampleColour(RevBlinkinLedDriver.BlinkinPattern.RED));
            }

            else if (gamepad1.dpad_down && !prevDown){
                CommandScheduler.getInstance().schedule(new SetPattern().blinkSampleColour(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_OCEAN_PALETTE));

            }

            prevUP = gamepad1.dpad_up;
            prevDown = gamepad1.dpad_down;

            telemetry.update();
        }
    }
}
