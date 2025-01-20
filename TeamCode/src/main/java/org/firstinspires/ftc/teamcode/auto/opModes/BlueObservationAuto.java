package org.firstinspires.ftc.teamcode.auto.opModes;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.auto.commands.factory.ObservationCommandFactory;
import org.firstinspires.ftc.teamcode.helpers.opmode.VLRLinearOpMode;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.claw.commands.SetClawTwist;

@Config
@Photon
@Autonomous(name = "BlueObservationAuto", group = "Blue Team")
public class BlueObservationAuto extends VLRLinearOpMode {
    @Override
    public void run() {
        AutoOpModeRunnner runner = new AutoOpModeRunnner(new ObservationCommandFactory(true), false);
        runner.initialize(hardwareMap);
        waitForStart();
        runner.run(this::opModeIsActive, false);
    }

}
