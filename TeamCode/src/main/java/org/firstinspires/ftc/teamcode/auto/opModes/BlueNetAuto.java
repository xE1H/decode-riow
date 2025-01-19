package org.firstinspires.ftc.teamcode.auto.opModes;

import com.acmerobotics.dashboard.config.Config;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.commands.factory.NetCommandFactory;
import org.firstinspires.ftc.teamcode.helpers.opmode.VLRLinearOpMode;

@Config
@Photon
@Autonomous(name = "BlueNetAuto", group = "Blue Team")
public class BlueNetAuto extends VLRLinearOpMode {

    @Override
    public void run() {
        AutoOpModeRunnner runner = new AutoOpModeRunnner(new NetCommandFactory(true), false);
        runner.initialize(hardwareMap);
        waitForStart();
        runner.run(this::opModeIsActive, false);
    }
}
