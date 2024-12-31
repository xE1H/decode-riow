package org.firstinspires.ftc.teamcode.auto.opModes;

import com.acmerobotics.dashboard.config.Config;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.commandFactory.NetCommandFactory;
import org.firstinspires.ftc.teamcode.helpers.opmode.VLRLinearOpMode;
import org.firstinspires.ftc.teamcode.subsystems.arm.rotator.ArmRotatorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;

@Config
@Photon
@Autonomous(name = "BlueNetAuto", group = "Blue Team")
public class BlueNetAuto extends VLRLinearOpMode {

    @Override
    public void run() {
        AutoOpModeRunnner runner = new AutoOpModeRunnner(new NetCommandFactory(true), false, ArmSlideSubsystem.class, ArmRotatorSubsystem.class, ClawSubsystem.class);
        runner.initialize(hardwareMap);
        waitForStart();
        runner.run();

        while (opModeIsActive()) {
            // This is a placeholder to keep the opmode running
        }
    }
}
