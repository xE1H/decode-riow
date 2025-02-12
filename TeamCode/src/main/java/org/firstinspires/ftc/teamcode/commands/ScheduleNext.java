package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.StrategyController;
import org.firstinspires.ftc.teamcode.helpers.enums.Alliance;

public class ScheduleNext extends InstantCommand {
    StrategyController controller;
    Follower f;
    Alliance alliance;

    public ScheduleNext(StrategyController controller, Follower f, Alliance alliance) {
        this.controller = controller;
        this.f = f;
        this.alliance = alliance;
    }

    @Override
    public void initialize() {
        CommandScheduler cs = CommandScheduler.getInstance();
        controller.reportAsCompleted();
        System.out.println("SCHEDULE NEXT: REPORTED AS COMPLETED");
        System.out.println("SCHEDULE NEXT: CURRENT STATE: " + controller.getCurrentState());

        switch (controller.getCurrentState()) {
            case INIT:
                // shouldn't even be here really.
                cs.schedule(new ScheduleNext(controller, f, alliance));
                break;
            case SPIKE_SCORE:
                cs.schedule(new ScheduleNext(controller, f, alliance));
                // todo run auto preload + 3 spike marks.
                break;
            case SUB_GRAB:
                cs.schedule(new SequentialCommandGroup(
                        new SubmersibleGrab(f, alliance),
                        new ScheduleNext(controller, f, alliance)
                ));
                break;
            case LOW_SCORE:
                // todo modify highbasketscore command to score into the low basket.
                // now it just runs the high_score function.

            case HIGH_SCORE:
                cs.schedule(new SequentialCommandGroup(
                        new HighBasketScore(f),
                        new ScheduleNext(controller, f, alliance)
                ));
                break;
            case HANG:
                // todo automatic hang
                break;
            case AUTO_TELEOP_TRANSITION_WAIT:
                cs.schedule(new SequentialCommandGroup(
                        new WaitCommand(8000),
                        new ScheduleNext(controller, f, alliance)
                ));
                break;
            case END:
                break;
        }
    }
}
