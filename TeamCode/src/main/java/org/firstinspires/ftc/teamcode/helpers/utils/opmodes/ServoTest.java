package org.firstinspires.ftc.teamcode.helpers.utils.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleRevHub;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleServo;

import org.firstinspires.ftc.teamcode.helpers.opmode.VLRLinearOpMode;

@Config
@TeleOp()
public class ServoTest extends VLRLinearOpMode {
    CuttleRevHub controlHub;
    CuttleServo servo0, servo1;

    public static boolean servo1State = false;
    public static boolean servo0State = false;

    @Override
    public void run() {
        controlHub = new CuttleRevHub(hardwareMap, CuttleRevHub.HubTypes.CONTROL_HUB);
        servo0 = new CuttleServo(controlHub, 0);
        servo1 = new CuttleServo(controlHub, 1);

        waitForStart();

        while(opModeIsActive()){
            if (gamepad1.triangle){
                servo0.setPosition(1);
            }
            else if (gamepad1.cross){
                servo0.setPosition(0);
            }

            else if (gamepad1.circle){
                servo1.setPosition(1);
            }

            else if (gamepad1.square){
                servo1.setPosition(0);
            }

            servo0.setPosition(servo0State ? 1 : 0);
            servo1.setPosition(servo1State ? 1 : 0);

            telemetry.update();
        }
    }
}