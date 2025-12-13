package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.ShooterProfiling.InterpLUT;
import org.firstinspires.ftc.teamcode.ShooterProfiling.ShooterPair;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Indexer.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Intake.Bootwheel;
import org.firstinspires.ftc.teamcode.subsystems.Outtake.Hood;
import org.firstinspires.ftc.teamcode.subsystems.Outtake.Shooter;
import org.firstinspires.ftc.teamcode.util.statemachine.State;
import org.firstinspires.ftc.teamcode.util.statemachine.StateMachine;
import org.firstinspires.ftc.teamcode.util.statemachine.Transition;

import java.util.ArrayList;
public class VoyagerXAuto {

    private Bootwheel intake;
    private Transfer transfer;
    private Shooter flywheel;
    private Hood hood;

    private Follower follower;
    private StateMachine state;
    private int curState;
    private boolean shootingInit = false;

    private final Pose goal = new Pose(129.5, 129.5);

    private final Pose startPose = new Pose(80.972, 7.85, Math.toRadians(90));
    private final Pose score = new Pose(81.421, 21.084, Math.toRadians(70));
    private final Pose intakeMid = new Pose(145, 59.644, Math.toRadians(0));
    private final Pose openGate = new Pose(135, 72, Math.toRadians(-90));
    private final Pose intakeFar = new Pose(145, 35, Math.toRadians(0));
    private final Pose intakeHP = new Pose(139, 10, Math.toRadians(-43));

    private PathChain scorePreload, intakeFirstSet, gateOpen, scoreFirstSet, intakeSecondSet, scoreSecondSet, pickupHP, scoreHP, park;

    private void buildPaths() {
        scorePreload = follower.pathBuilder()
                .addPath(
                        new BezierLine(startPose, score)
                )
                .setLinearHeadingInterpolation(startPose.getHeading(), score.getHeading())
                .build();

        intakeFirstSet = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                score,
                                new Pose(73.346, 61.682),
                                new Pose(97.346, 58.542)
                        )
                )
                .addPath(
                        new BezierLine(
                                new Pose(97.346, 58.542),
                                intakeMid
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        gateOpen = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                intakeMid,
                                new Pose(111.925, 72.224),
                                openGate
                        )
                )
                .setLinearHeadingInterpolation(intakeMid.getHeading(), openGate.getHeading())
                .build();

        scoreFirstSet = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                openGate,
                                new Pose(61.009, 70.654),
                                score
                        )
                )
                .setLinearHeadingInterpolation(openGate.getHeading(), score.getHeading())
                .build();

        intakeSecondSet = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                score,
                                new Pose(72.449, 37.009),
                                intakeFar
                        )
                )
                .setLinearHeadingInterpolation(score.getHeading(), intakeFar.getHeading())
                .build();

        scoreSecondSet = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                intakeFar,
                                new Pose(68.411, 39.925),
                                score
                        )
                )
                .setLinearHeadingInterpolation(intakeFar.getHeading(), score.getHeading())
                .build();

        pickupHP = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                score,
                                new Pose(88.150, 50.243),
                                new Pose(130.991, 17.495)
                        )
                )
                .addPath(
                        new BezierLine(
                                new Pose(130.991, 17.495),
                                intakeHP
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        scoreHP = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                intakeHP,
                                new Pose(93.981, 51.813),
                                score
                        )
                )
                .setLinearHeadingInterpolation(intakeHP.getHeading(), score.getHeading())
                .build();

        park = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                score,
                                new Pose(85, 25)
                        )
                )
                .setLinearHeadingInterpolation(score.getHeading(), intakeFar.getHeading())
                .build();
    }

    private State[] stateBuilder() {
        ArrayList<State> temp = new ArrayList<>();

        temp.add(
                new State("PRELOAD")
                        .setEntry(() -> {
                            intake.setPower(1);
                            follower.followPath(scorePreload);
                            curState = 0;
                        })
                        .addTransition(new Transition(() -> !follower.isBusy(), "SCORE"))
        );

        temp.add(
                new State("INTAKE_FIRST")
                        .setEntry(() -> {
                            follower.followPath(intakeFirstSet);
                            curState = 1;
                        })
                        .setExit(() -> {
                            intake.setPower(1);
                        })
                        .addTransition(new Transition(() -> !follower.isBusy(), () -> nextPath(curState)))
        );

        temp.add(
                new State("OPEN_GATE")
                        .setEntry(() -> {
                            follower.followPath(gateOpen);
                            curState = 2;
                        })
                        .addTransition(new Transition(() -> !follower.isBusy(), () -> nextPath(curState)))
        );

        temp.add(
                new State("SCORE_FIRST")
                        .setEntry(() -> {
                            follower.followPath(scoreFirstSet);
                            curState = 3;
                        })
                        .addTransition(new Transition(() -> !follower.isBusy(), "SCORE"))
        );

        temp.add(
                new State("INTAKE_SECOND")
                        .setEntry(() -> {
                            follower.followPath(intakeSecondSet);
                            curState = 4;
                        })
                        .addTransition(new Transition(() -> !follower.isBusy(), () -> nextPath(curState)))
        );

        temp.add(
                new State("SCORE_SECOND")
                        .setEntry(() -> {
                            follower.followPath(scoreSecondSet);
                            curState = 5;
                        })
                        .addTransition(new Transition(() -> !follower.isBusy(), "SCORE"))
        );

        temp.add(
                new State("INTAKE_HP")
                        .setEntry(() -> {
                            follower.followPath(pickupHP);
                            curState = 6;
                        })
                        .addTransition(new Transition(() -> !follower.isBusy(), () -> nextPath(curState)))
        );

        temp.add(
                new State("SCORE_HP")
                        .setEntry(() -> {
                            follower.followPath(scoreHP);
                            curState = 7;
                        })
                        .addTransition(new Transition(() -> !follower.isBusy(), "SCORE"))
        );

        temp.add(
                new State("PARK")
                        .setEntry(() -> {
                            follower.followPath(park);
                        })
                        .setDuring(() -> {
                            jankTele.target = goal;
//                            jankTele.init = follower.getPose();
                        })
        );

        temp.add(
                new State("SCORE")
                        .setEntry(() -> {
                            shootingInit = true;
                            transfer.resetFlags();
                            transfer.requestShoot();
                        })
                        .setDuring(() -> {
                            if (flywheel.atVelocity()) {
                                transfer.requestReady();
                            }
                        })
                        .setExit(() -> {
                            shootingInit = false;
                            intake.setPower(1);
                        })
                        .setMinTime(200)
                        .addTransition(new Transition(
                                () -> shootingInit && !transfer.isShooting(),
                                () -> nextPath(curState)
                        ))
        );

        return temp.toArray(new State[0]);
    }

    private String nextPath (int curState) {
        switch (curState) {
            case 0: return "INTAKE_FIRST";
            case 1: return "OPEN_GATE";
            case 2: return "SCORE_FIRST";
            case 3: return "INTAKE_SECOND";
            case 4: return "SCORE_SECOND";
            case 5: return "INTAKE_HP";
            case 6: return "SCORE_HP";
            case 7: return "PARK";
            default: return "ERROR";
        }
    }

    @Override
    public void init() {
        intake = new Bootwheel(hardwareMap);
        transfer = new Transfer(hardwareMap);
        flywheel = new Shooter(hardwareMap);
        hood = new Hood(hardwareMap);

        transfer.init();
        hood.setPosition(0.536);

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

        state = new StateMachine(stateBuilder());
        state.start();
    }

    @Override
    public void loop() {
        state.run();
        follower.update();

        flywheel.update();
        hood.update();
        intake.update();
        transfer.update();

        // autoSet shooter

        flywheel.setTargetVel(1400);
        hood.setPosition(.536);

        telemetry.addData("state", state.currentState());
        telemetry.addData("robot pose", follower.getPose().toString());
        telemetry.addData("transfer", transfer.toString());
        telemetry.addData("Target Vel", flywheel.getTargetVel());
        telemetry.addData("Current Vel", flywheel.getCurrentVel());
        telemetry.addData("At Velocity?", flywheel.atVelocity());

        telemetry.update();
    }

}
