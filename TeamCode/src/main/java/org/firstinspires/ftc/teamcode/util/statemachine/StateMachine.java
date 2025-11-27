package org.firstinspires.ftc.teamcode.util.statemachine;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class StateMachine {
    private List<State> states;
    private int index;
    private boolean isOn;

    public StateMachine(State[] states) {
        this.states = new ArrayList<>(Arrays.asList(states));
        this.index = 0;
        isOn = false;
    }

    public void start(int startIndex) {
        if (!states.isEmpty()) {
            index = startIndex;
            isOn = true;
            states.get(startIndex).start();
        }
    }

    public void start() {
        start(0);
    }

    public void run() {
        if (!isOn) return;

        State cur = states.get(index);
        cur.run();

        if (cur.isFinished()) {
            String nStateString = cur.getNextState();
            int nIndex = (nStateString != null) ? getIndex(nStateString) : index + 1;

            if (nIndex >= states.size() || nIndex < 0) {
                isOn = false;
            } else {
                index = nIndex;
                states.get(nIndex).start();
            }
        }
    }

    private int getIndex(String stateName) {
        for (int i = 0; i < states.size(); i++) {
            if (states.get(i).getName().equals(stateName)) return i;
        }
        return -1;
    }

    public String currentState() {
        return states.get(index).getName();
    }

    public boolean isFinished() {
        return !isOn;
    }
}