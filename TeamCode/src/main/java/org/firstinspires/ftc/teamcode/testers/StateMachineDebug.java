package org.firstinspires.ftc.teamcode.testers;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Indexer.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Indexer.Spindex;
import org.firstinspires.ftc.teamcode.util.statemachine.State;
import org.firstinspires.ftc.teamcode.util.statemachine.StateMachine;
import org.firstinspires.ftc.teamcode.util.statemachine.Transition;

import java.util.function.BooleanSupplier;

@Disabled
@TeleOp(name = "testStateMachine", group = "tuners")
public class StateMachineDebug extends OpMode {

    private Spindex spindex;
    private Elevator transfer;
    private StateMachine state;
    private int curState;
    private String curBall = "CENTER";

    @Override
    public void init() {
        spindex = new Spindex(hardwareMap);
        transfer = new Elevator(hardwareMap);

        BooleanSupplier left = () -> gamepad2.dpadLeftWasPressed();
        BooleanSupplier right = () -> gamepad2.dpadRightWasPressed();
        BooleanSupplier center = () -> gamepad2.dpadUpWasPressed();
        BooleanSupplier transferRequest = () -> gamepad2.dpadDownWasPressed();

        State[] temp = new State[5];
        temp[0] = new State("CENTER")
                .setDuring(() -> {
                    spindex.setPosition(Constants.Positions.CENTER);
                    curState = 2;
                })
                .addTransition(new Transition(left, "LEFT"))
                .addTransition(new Transition(right, "RIGHT"))
                .addTransition(new Transition(transferRequest, "DOWN"));
        temp[1] = new State("LEFT")
                .setDuring(() -> {
                    spindex.setPosition(Constants.Positions.LEFT);
                    curState = 1;
                })
                .addTransition(new Transition(center, "CENTER"))
                .addTransition(new Transition(right, "RIGHT"))
                .addTransition(new Transition(transferRequest, "DOWN"));
        temp[2] = new State("RIGHT")
                .setDuring(() -> {
                    spindex.setPosition(Constants.Positions.RIGHT);
                    curState = 3;
                })
                .addTransition(new Transition(center, "CENTER"))
                .addTransition(new Transition(left, "LEFT"))
                .addTransition(new Transition(transferRequest, "DOWN"));
        temp[3] = new State("DOWN")
                .setDuring(() -> {
                    transfer.setStatus(Constants.ElevatorPosition.DOWN);
                })
                .setMinTime(350)
                .setMaxTime(350)
                .setFallbackState("UP");
        temp[4] = new State("UP")
                .setDuring(() -> {

                    switch (curState) {
                        case 1:
                            curBall = "LEFT";
                            break;
                        case 2:
                            curBall = "CENTER";
                            break;
                        case 3:
                            curBall = "RIGHT";
                            break;
                    }
                    transfer.setStatus(Constants.ElevatorPosition.UP);
                })
                .setExit(() -> {
                    temp[4].setFallbackState(curBall);
                })
                .setMinTime(350)
                .setMaxTime(350)
                .setFallbackState("CENTER");

        state = new StateMachine(temp);
        state.start();
    }

    @Override
    public void loop() {
        state.run();
        transfer.update();
        spindex.update();
    }
}
