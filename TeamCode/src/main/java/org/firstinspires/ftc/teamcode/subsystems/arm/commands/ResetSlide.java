//package org.firstinspires.ftc.teamcode.subsystems.arm.commands;
//
//import com.arcrobotics.ftclib.command.ParallelRaceGroup;
//import com.arcrobotics.ftclib.command.WaitCommand;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.helpers.commands.InstantCommand;
//import org.firstinspires.ftc.teamcode.helpers.subsystems.VLRSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.arm.slide.ArmSlideSubsystem;
//
//public class ResetSlide extends ParallelRaceGroup {
//    public ResetSlide(){
//        addCommands(
//                new InstantCommand() {
//                    @Override
//                    public void run() {
//                        ArmSlideSubsystem ass = VLRSubsystem.getInstance(ArmSlideSubsystem.class);
//                        ass.setMotorPower(-0.2);
//                        while(!ass.getLimitSwitchState()){
//                            try {
//                                Thread.sleep(1);
//                            } catch (InterruptedException e) {}
//                        }
//                        ass.setMotorPower(0);
//                        ass.checkLimitSwitch();
//                    }
//                },
//                new WaitCommand(1000)
//        );
//    }
//}
