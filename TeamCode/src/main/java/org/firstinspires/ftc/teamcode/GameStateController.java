package org.firstinspires.ftc.teamcode;

import java.util.function.Supplier;

public class GameStateController {
    //    10.1 MATCH Overview
    //    MATCHES run on a typical 6-10-minute cycle time per FIELD, which consists of pre-MATCH setup, a 30 second
    //    AUTO period, an 8 second transition period between AUTO and TELEOP, and a 2-minute TELEOP period,
    //    followed by the post-MATCH reset.
    private Supplier<Double> rawTimePassed;
    private boolean postInit = false;

    public enum GameStage {
        PRE_MATCH,
        AUTO,
        TRANSITION,
        TELEOP,
        END
    }

    public GameStateController(Supplier<Double> timePassed, boolean forceTeleOp) {
        if (forceTeleOp) {
            this.rawTimePassed = () -> 40.0 + timePassed.get();
        } else {
            this.rawTimePassed = timePassed;
        }
    }

    public void postInit() {
        postInit = true;
    }

    public GameStage getGameStage() {
        if (!postInit) {
            return GameStage.PRE_MATCH;
        }

        double timePassed = rawTimePassed.get();
        if (timePassed < 30) {
            return GameStage.AUTO;
        } else if (timePassed < 40) {
            return GameStage.TRANSITION;
        } else if (timePassed < 160) {
            return GameStage.TELEOP;
        } else {
            return GameStage.END;
        }
    }

    public double getCorrectedTimePassed() {
        double timePassed = rawTimePassed.get();
        if (!postInit) return timePassed;

        if (timePassed < 30) return timePassed;
        else if (timePassed < 40) return timePassed - 30;
        else if (timePassed < 160) return timePassed - 40;
        else return timePassed - 160;
    }
}
