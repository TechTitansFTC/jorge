package org.firstinspires.ftc.teamcode.util.statemachine;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class Transition {

    private boolean isRunning;
    private final BooleanSupplier condition;
    private final Supplier<String> nextState;

    public Transition(BooleanSupplier condition, String nextState) {
        this(condition, () -> nextState);
    }

    public Transition(BooleanSupplier condition, Supplier<String> nextState) {
        this.condition = condition;
        this.nextState = nextState;
        isRunning = false;
    }

    public void start() {
        isRunning = true;
    }

    public void run() {
        if (isRunning && condition.getAsBoolean()) {
            isRunning = false;
        }
    }

    public String getNextState() {
        return nextState.get();
    }

    public boolean isFinished() {
        return !isRunning;
    }
}
