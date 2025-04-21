package org.firstinspires.ftc.teamcode.helpers.opmode;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
import org.firstinspires.ftc.teamcode.helpers.utils.GlobalConfig;
import org.firstinspires.ftc.teamcode.subsystems.arm.MainArmConfiguration;
import org.firstinspires.ftc.teamcode.subsystems.arm.MainArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.chassis.Chassis;
import org.firstinspires.ftc.teamcode.subsystems.claw.ClawSubsystem;

public abstract class VLRTestOpMode extends VLRLinearOpMode{

    @Override
    public void run(){
        VLRSubsystem.requireSubsystems(MainArmSubsystem.class, ClawSubsystem.class, Chassis.class);
        VLRSubsystem.initializeAll(hardwareMap);

        GlobalConfig.DEBUG_MODE = true;
        CommandScheduler.getInstance().schedule(new InstantCommand(()-> VLRSubsystem.getArm().setOperationMode(MainArmConfiguration.OPERATION_MODE.NORMAL)));

        Init();
        while (opModeInInit()) {InitLoop();}

        waitForStart();

        Start();
        while (opModeIsActive()) {Loop();}

        Stop();
    }

    public abstract void Loop();
    public void Init() {}
    public void Start() {}
    public void Stop(){}
    public void InitLoop(){}
}
