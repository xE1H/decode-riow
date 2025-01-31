package org.firstinspires.ftc.teamcode.auto.opModes;

import com.acmerobotics.dashboard.config.Config;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.AutoOpModeRunner;
import org.firstinspires.ftc.teamcode.auto.commands.factory.NetCommandFactory;
import org.firstinspires.ftc.teamcode.helpers.opmode.VLRLinearOpMode;

@Config
@Photon
@Autonomous(name = "NetAuto", group = "!AUTO")
public class NetAuto extends VLRLinearOpMode {

    @Override
    public void run() {
        AutoOpModeRunner runner = new AutoOpModeRunner(new NetCommandFactory());
        runner.initialize(hardwareMap);
        waitForStart();
        runner.run(this::opModeIsActive, false);
    }
}
