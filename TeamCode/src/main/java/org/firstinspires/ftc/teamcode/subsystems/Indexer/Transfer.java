package org.firstinspires.ftc.teamcode.subsystems.Indexer;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.util.statemachine.State;
import org.firstinspires.ftc.teamcode.util.statemachine.StateMachine;
import org.firstinspires.ftc.teamcode.util.statemachine.Transition;

import java.util.LinkedList;
import java.util.Queue;
import java.util.function.BooleanSupplier;

public class Transfer {

    private StateMachine state;

    private Elevator elevator;
    private Spindex spindex;

    private boolean requestShot = false, isReady = false;
    private BooleanSupplier shoot, ready, done;

    private Queue<Integer> shotQ = new LinkedList<>();
    private String curBall = "LEFT";
    private String targetSlot = "LEFT";
    private int curSlot;
    private final int timeForSpindex = 500;
    private boolean hasOne, hasTwo, hasThree;

    public Transfer (HardwareMap map) {
        elevator = new Elevator(map);
        spindex = new Spindex(map);

        curSlot = 1;

        hasOne = false;
        hasTwo = false;
        hasThree = false;

        shoot = () -> requestShot;
        ready = () -> isReady;
        done = () -> !requestShot;

        State[] states = createStates();
        state = new StateMachine(states);
    }

    private State[] createStates() {
        State[] states = new State[7];


        states[0] = new State("IDLE")
                .setDuring(() -> {
                    elevator.setStatus(Constants.ElevatorPosition.UP);
                    spindex.setPosition(Constants.Positions.LEFT);
                    curSlot = 2;
                })
                .addTransition(new Transition(shoot, "WAITING"));

        states[1] = new State("WAITING")
                .setEntry(() -> {
                    // here is where we will impl sorting logic at some point
                    // targetSlot is the one we want to go to next

                    if (!shotQ.isEmpty()) {
                        Integer next = shotQ.remove();
                        if ((next != null) && hasBall(next)) {
                            targetSlot = getSlotName(next);
                            curSlot = next;
                        }
                    } else {
                        if (hasOne && curSlot != 1) {
                            targetSlot = "LEFT";
                            curSlot = 1;
                        } else if (hasTwo && curSlot != 2) {
                            targetSlot = "CENTER";
                            curSlot = 2;
                        } else if (hasThree && curSlot != 3) {
                            targetSlot = "RIGHT";
                            curSlot = 3;
                        } else {
                            targetSlot = "IDLE";
                        }
                    }

                    requestShot = !targetSlot.equals("IDLE");

                })
                .addTransition(new Transition(shoot, () -> targetSlot))
                .addTransition(new Transition(done, "IDLE"))
                .setMaxTime(0);

        states[2] = new State("LEFT")
                .setEntry(() -> {
                    spindex.setPosition(Constants.Positions.LEFT);
                    curSlot = 1;
                })
                .setMinTime(timeForSpindex)
                .addTransition(new Transition(() -> ready.getAsBoolean() && hasOne, "DOWN"))
                .setFallbackState("WAITING")
                .setMaxTime(timeForSpindex); // jank to time based while I debug nServo

        states[3] = new State("CENTER")
                .setEntry(() -> {
                    spindex.setPosition(Constants.Positions.CENTER);
                    curSlot = 2;
                    if (!hasTwo) {
                        states[3].setFallbackState("WAITING");
                        states[3].setMaxTime(0);
                    }
                })
                .setMinTime(timeForSpindex)
                .setMaxTime(timeForSpindex)
                .setFallbackState("WAITING")
                .addTransition(new Transition(() -> ready.getAsBoolean() && hasTwo, "DOWN"));

        states[4] = new State("RIGHT")
                .setEntry(() -> {
                    spindex.setPosition(Constants.Positions.RIGHT);
                    curSlot = 3;
                    if (!hasThree) {
                        states[4].setFallbackState("WAITING");
                        states[4].setMaxTime(0);
                    }
                })
                .setMinTime(timeForSpindex)
                .setMaxTime(timeForSpindex)
                .setFallbackState("WAITING")
                .addTransition(new Transition(() -> ready.getAsBoolean() && hasThree, "DOWN"));

        states[5] = new State("DOWN")
                .setEntry(() -> elevator.setStatus(Constants.ElevatorPosition.DOWN))
                .setFallbackState("UP")
                .setMinTime(400)
                .setMaxTime(400);

        states[6] = new State("UP")
                .setEntry(() -> {
                    elevator.setStatus(Constants.ElevatorPosition.UP);
                    curBall = (curSlot == 1) ? "LEFT" : (curSlot == 2) ? "CENTER" : "RIGHT";
                })
                .setExit(() -> {
                    isReady = false;

                    if (curSlot == 1) hasOne = false;
                    else if (curSlot == 2) hasTwo = false;
                    else if (curSlot == 3) hasThree = false;

                    if (!hasOne && !hasTwo && !hasThree) {
                        requestShot = false;
                    }

                    states[6].setFallbackState("WAITING");
                })
                .setFallbackState("WAITING")
                .setMinTime(100)
                .setMaxTime(100);

        return states;
    }

    public void init() {
        state.start();
        elevator.setStatus(Constants.ElevatorPosition.UP);
        spindex.setPosition(Constants.Positions.LEFT);
    }

    public void update() {
        elevator.update();
        spindex.update();

        state.run();
    }

    public void requestShoot(int first, int second, int third) {
        if (first >= 1 && first <= 3) shotQ.add(first);
        if (second >= 1 && second <= 3) shotQ.add(second);
        if (third >= 1 && third <= 3) shotQ.add(third);

        requestShot = true;
    }

    public void requestShoot() {
        shotQ.clear();

        requestShot = true;
    }

    private boolean hasBall (int slot) {
        switch (slot) {
            case 1: return hasOne;
            case 2: return hasTwo;
            case 3: return hasThree;
            default: return false;
        }
    }

    private String getSlotName (int slot) {
        switch (slot) {
            case 1: return "LEFT";
            case 2: return "CENTER";
            case 3: return "RIGHT";
            default: return "IDLE";
        }
    }

    public void requestReady() {
        isReady = true;
    }

    public boolean isShooting() {
        return !state.currentState().equals("IDLE");
    }

    public void resetFlags() {
        hasOne = true;
        hasTwo = true;
        hasThree = true;
    }

    @Override
    public String toString() {
        return "State: " + state.currentState() + "\n" +
                "curSlot: " + curSlot + "\n" +
                "targetSlot: " + targetSlot + "\n" +
                "hasOne: " + hasOne + "\n" +
                "hasTwo: " + hasTwo + "\n" +
                "hasThree: " + hasThree + "\n" +
                "requestShot: " + requestShot + "\n" +
                "isReady: " + isReady + "\n";

    }
}
