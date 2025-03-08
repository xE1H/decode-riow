package org.firstinspires.ftc.teamcode;

public class StrategyController {
    public final GameStateController gameStateController;
    private State currentState;
    private State previousState;
    private int highScoreCycles = 0;

    public enum State {
        INIT,
        SPIKE_SCORE,
        AUTO_TELEOP_TRANSITION_WAIT,
        SUB_GRAB,
        HIGH_SCORE,
        LOW_SCORE,
        HANG,
        END
    }

    public StrategyController(GameStateController gameStateController) {
        this.gameStateController = gameStateController;
        currentState = State.INIT;
        previousState = State.INIT;
    }

    public State getCurrentState() {
        return currentState;
    }

    public State getPreviousState() {
        return previousState;
    }

    public void reportAsCompleted() {
        determineNextState();
    }

    private void determineNextState() {
        if (currentState == State.END || gameStateController.getGameStage() == GameStateController.GameStage.END) {
            currentState = State.END;
            return;
        }

        if (gameStateController.getGameStage() == GameStateController.GameStage.TRANSITION) {
            previousState = currentState;
            currentState = State.AUTO_TELEOP_TRANSITION_WAIT;
            return;
        }

        if (gameStateController.getGameStage() == GameStateController.GameStage.TELEOP) {
            double teleopTime = gameStateController.getCorrectedTimePassed();
//            if (120 - teleopTime < 15) {
//                currentState = State.HANG;
//                return;
//            }
        }

        switch (currentState) {
            case INIT:
                switch (gameStateController.getGameStage()) {
                    case AUTO:
                        currentState = State.SPIKE_SCORE;
                        break;
                    case TRANSITION:
                        currentState = State.AUTO_TELEOP_TRANSITION_WAIT;
                        break;
                    case TELEOP:
                        currentState = State.SUB_GRAB;
                        break;
                    case PRE_MATCH:
                        break;
                }
                break;
            case SPIKE_SCORE:
                if (gameStateController.getCorrectedTimePassed() > 10) {
                    currentState = State.SUB_GRAB;
                }
                break;
            case AUTO_TELEOP_TRANSITION_WAIT:
                if (gameStateController.getGameStage() == GameStateController.GameStage.TELEOP) {
                    currentState = State.SUB_GRAB;
                    highScoreCycles = 0;
                }
                break;
            case SUB_GRAB:
                if (gameStateController.getGameStage() == GameStateController.GameStage.AUTO) {
                    double autoTime = gameStateController.getCorrectedTimePassed();
                    currentState = State.HIGH_SCORE;
                } else if (gameStateController.getGameStage() == GameStateController.GameStage.TELEOP) {
                    if (highScoreCycles < 100) {
                        currentState = State.HIGH_SCORE;
                    } else {
                        currentState = State.LOW_SCORE;
                    }
                }
                break;
            case HIGH_SCORE:
                if (gameStateController.getGameStage() == GameStateController.GameStage.AUTO) {
                    currentState = State.AUTO_TELEOP_TRANSITION_WAIT;
                } else if (gameStateController.getGameStage() == GameStateController.GameStage.TELEOP) {
                    highScoreCycles++;
                    currentState = State.SUB_GRAB;
                }
                break;
            case LOW_SCORE:
                currentState = State.SUB_GRAB;
                break;
            case HANG:
                currentState = State.HANG;
                break;
            case END:
                break;
        }
    }
}