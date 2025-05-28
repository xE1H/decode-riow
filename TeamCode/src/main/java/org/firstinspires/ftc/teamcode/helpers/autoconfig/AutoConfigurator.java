package org.firstinspires.ftc.teamcode.helpers.autoconfig;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Arrays;
import java.util.Collections;
import java.util.function.BooleanSupplier;
import java.util.logging.Logger;

public class AutoConfigurator {
    private final Telemetry telemetry;
    private final Gamepad gamepad;
    private static final Logger logger = Logger.getLogger("AutoConfigurator");
    private final BooleanSupplier isStopRequested;

    public static class Choice {
        public String text;
        public Runnable action;

        public Choice(String text, Runnable action) {
            this.text = text;
            this.action = action;
        }

        public Choice(String text) {
            this.text = text;
            this.action = () -> logger.info("picked" + text);
        }
    }

    public AutoConfigurator(Telemetry telemetry, Gamepad gamepad, BooleanSupplier isStopRequested) {
        this.telemetry = telemetry;
        this.gamepad = gamepad;
        this.isStopRequested = isStopRequested;
    }

    public Choice multipleChoice(String question, Choice... choices) {
        if (choices.length == 0) {
            throw new IllegalArgumentException("Must have at least one choice");
        }
        Choice currentChoice = choices[0];
        boolean accepted = false;
        boolean latched = false;

        long startTime = System.currentTimeMillis();
        boolean isAccepting = false;

        while (!accepted && !isStopRequested.getAsBoolean()) {
            telemetry.addLine(question);

            for (Choice choice : choices) {
                if (choice == currentChoice) {
                    telemetry.addLine(String.format("> %s", choice.text));
                } else {
                    telemetry.addLine(String.format("  %s", choice.text));
                }
            }

            telemetry.addLine();
            telemetry.addLine("Use left stick to navigate up/down.");
            telemetry.addLine("Hold left stick to the right to accept.");
            if (isAccepting) {

                int acceptProgress = (int) Math.round((System.currentTimeMillis() - startTime) / 50.0);
                telemetry.addLine();
                // some java bs to repeat a string n times...
                telemetry.addLine("Accepting [" +
                        String.join("", Collections.nCopies(acceptProgress, "=")) +
                        String.join("", Collections.nCopies(10 - acceptProgress, "_")) +
                        "]");
            }

            telemetry.update();

            if (gamepad.left_stick_y < -0.5) {
                // Move up
                if (!latched) {
                    latched = true;
                    int currentIndex = 0;

                    for (int i = 0; i < choices.length; i++) {
                        if (choices[i] == currentChoice) {
                            currentIndex = i;
                            break;
                        }
                    }
                    currentIndex = (currentIndex - 1 + choices.length) % choices.length;
                    currentChoice = choices[currentIndex];
                    gamepad.rumbleBlips(1);
                }
            } else if (gamepad.left_stick_y > 0.5) {
                // Move down
                if (!latched) {
                    latched = true;
                    int currentIndex = 0;
                    for (int i = 0; i < choices.length; i++) {
                        if (choices[i] == currentChoice) {
                            currentIndex = i;
                            break;
                        }
                    }
                    currentIndex = (currentIndex + 1) % choices.length;
                    currentChoice = choices[currentIndex];
                    gamepad.rumbleBlips(1);
                }
            } else {
                latched = false;
            }
            // To accept - hold stick right for 1 second.
            if (gamepad.left_stick_x > 0.5) {
                isAccepting = true;
                double timeElapsed = System.currentTimeMillis() - startTime;
                double strength = Math.min(1, timeElapsed / 500);
                gamepad.rumble(strength, strength, Gamepad.RUMBLE_DURATION_CONTINUOUS);
                if (timeElapsed >= 500) {
                    accepted = true;
                }
            } else {
                gamepad.rumble(0, 0, Gamepad.RUMBLE_DURATION_CONTINUOUS);
                startTime = System.currentTimeMillis();
                isAccepting = false;
            }
        }

        gamepad.rumble(0, 0, 0);
        telemetry.clearAll();
        telemetry.update();

        return currentChoice;
    }

    public void review(String... lines) {
        telemetry.clearAll();
        // Accept - hold stick right for 1 second.
        long startTime = System.currentTimeMillis();

        while (!isStopRequested.getAsBoolean()) {
            telemetry.addLine("Review:");
            Arrays.stream(lines).forEach(telemetry::addLine);

            if (gamepad.left_stick_x > 0.5) {
                double timeElapsed = System.currentTimeMillis() - startTime;
                double strength = Math.min(1, timeElapsed / 1000);
                gamepad.rumble(strength, strength, Gamepad.RUMBLE_DURATION_CONTINUOUS);
                if (timeElapsed >= 1000) {
                    break;
                }
                telemetry.addLine("Accepting [" +
                        String.join("", Collections.nCopies((int) (Math.round(timeElapsed / 100.0)), "=")) +
                        String.join("", Collections.nCopies(10 - (int) (Math.round(timeElapsed / 100.0)), "_")) +
                        "]");
            } else {
                gamepad.rumble(0, 0, Gamepad.RUMBLE_DURATION_CONTINUOUS);
                startTime = System.currentTimeMillis();
            }
            telemetry.update();
        }

        gamepad.rumble(0, 0, 0);
        telemetry.clearAll();

        telemetry.addLine("Review:");
        Arrays.stream(lines).forEach(telemetry::addLine);
        telemetry.update();
    }
}