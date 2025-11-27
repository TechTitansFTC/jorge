package org.firstinspires.ftc.teamcode.subsystems.Intake;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.statemachine.State;
import org.firstinspires.ftc.teamcode.util.statemachine.StateMachine;
import org.firstinspires.ftc.teamcode.util.statemachine.Transition;

import java.util.function.BooleanSupplier;

public class Intake {

    private Bootwheel intakeMotor;
    private StateMachine state;
    private BooleanSupplier on, off, out;
    private boolean onBool = false, offBool = false, outBool = false;

    public Intake (HardwareMap map) {
        intakeMotor = new Bootwheel(map);

        State[] states = createStates();
        state = new StateMachine(states);
    }

    private State[] createStates () {
        State[] states = new State[3];

        states[0] = new State("OFF");

        states[0].setEntry(() -> intakeMotor.setPower(0))
                .addTransition(new Transition(on, "ON"))
                .addTransition(new Transition(out, "OUT"))
                .setExit(() -> offBool = false);

        states[1] = new State("ON");

        states[1].setEntry(() -> intakeMotor.setPower(1))
                .addTransition(new Transition(off, "OFF"))
                .setExit(() -> onBool = false);

        states[2] = new State("OUT");

        states[2].setEntry(() -> intakeMotor.setPower(-1))
                .addTransition(new Transition(off, "OFF"))
                .setExit(() -> outBool = false);

        return states;
    }

    public void init() {
        on = () -> onBool;
        off = () -> offBool;
        out = () -> outBool;

        state.start(0);
    }

    public void update() {
        intakeMotor.update();

        state.run();
    }

    public void requestOn() {
        onBool = true;
    }
    public void requestOut() {
        outBool = true;
    }
    public void requestOff() {
        offBool = true;
    }
}
