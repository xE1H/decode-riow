package org.firstinspires.ftc.teamcode.helpers.utils.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleAnalog;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleRevHub;
import com.roboctopi.cuttlefishftcbridge.devices.CuttleServo;


@Config
//@Disabled
@TeleOp(name="AxonMicroCalibration", group="Utils")
public class AxonCalibration extends OpMode
{
    private CuttleServo servo0;
    private CuttleServo servo1;
    private CuttleServo servo2;
    private CuttleServo servo3;
    private CuttleServo servo4;

    private CuttleRevHub controlHub;
    private CuttleRevHub expansionHub;

    private AnalogInput analog0;
    private AnalogInput analog1;

    public static double position0 = 0;
    public static double position1 = 0;
    public static double position2 = 0;
    public static double position3 = 0;
    public static double position4 = 0;


    @Override
    public void init() {
        //expansionHub = new CuttleRevHub(hardwareMap, "Expansion Hub 2");
        controlHub = new CuttleRevHub(hardwareMap, CuttleRevHub.HubTypes.CONTROL_HUB);

        servo0 = new CuttleServo(controlHub, 1);
        servo1 = new CuttleServo(controlHub, 2); //twist
        servo2 = new CuttleServo(controlHub, 3); //angle
        servo3 = new CuttleServo(controlHub, 4);
        servo4 = new CuttleServo(controlHub, 5); //claw

        analog0 = hardwareMap.get(AnalogInput.class, "analog0");
        analog1 = hardwareMap.get(AnalogInput.class, "analog1");
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        servo0.setPosition(position0);
        servo1.setPosition(position1);
        servo2.setPosition(position2);
        servo3.setPosition(position3);
        servo4.setPosition(position4);

        telemetry.addData("0", analog0.getVoltage());
        telemetry.addData("1", analog1.getVoltage());

    }
}
