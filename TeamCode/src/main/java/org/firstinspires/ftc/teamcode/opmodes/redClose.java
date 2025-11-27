package org.firstinspires.ftc.teamcode.opmodes;

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

@Autonomous(name = "red Close", group = "Autonomous")
public class redClose extends OpMode {

    private Bootwheel intake;
    private Transfer transfer;
    private Shooter flywheel;
    private Hood hood;
    private InterpLUT table;
    private Pose curPos;
    private double curDistance;

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private StateMachine state;
    private int curState;
    private boolean shootingInit = false;

    private final Pose goal = new Pose(129.5, 129.5);

    private final Pose startPose = new Pose(122.838, 123.081, Math.toRadians(36));
    private final Pose scorePose = new Pose(84.405, 84.649, Math.toRadians(36));
    private final Pose openGate = new Pose(134.757, 69.568, Math.toRadians(0));
    private final Pose intakeMid = new Pose(134.501, 59.082, Math.toRadians(0));
    private final Pose intakeFar = new Pose(134.501, 35.145, Math.toRadians(0));
    private final Pose parkPose = new Pose(98, 121, Math.toRadians(65));

    private PathChain scorePreload, intakeFirstSet, scoreFirstSet, intakeSecondSet, scoreSecondSet, intakeThirdSet, scoreThirdSet, park;

    private void buildPaths() {
        scorePreload = follower.pathBuilder()
             .addPath(
                     new BezierLine(startPose, scorePose)
             )
             .setConstantHeadingInterpolation(scorePose.getHeading())
             .build();

        intakeFirstSet = follower.pathBuilder()
             .addPath(
                     new BezierCurve(
                             scorePose,
                             new Pose(127.662, 90.237),
                             new Pose(139.441, 85.108),
                             new Pose(111.894, 73.900),
                             openGate
                     )
             )
             .setTangentHeadingInterpolation()
             .build();

        scoreFirstSet = follower.pathBuilder()
             .addPath(
                     new BezierCurve(
                             openGate,
                             new Pose(96.324, 64.216),
                             scorePose
                     )
             )
             .setLinearHeadingInterpolation(openGate.getHeading(), scorePose.getHeading())
             .build();

        intakeSecondSet = follower.pathBuilder()
                .addPath(new BezierCurve(
                        scorePose,
                        new Pose(105.568, 53.514),
                        intakeMid
                ))
                .setTangentHeadingInterpolation()
                .build();

        scoreSecondSet = follower.pathBuilder()
                .addPath(new BezierCurve(
                        intakeMid,
                        new Pose(103.622, 54.973),
                        scorePose
                ))
                .setLinearHeadingInterpolation(intakeMid.getHeading(), scorePose.getHeading())
                .build();

        intakeThirdSet = follower.pathBuilder()
                .addPath(new BezierCurve(
                        scorePose,
                        new Pose(86.108, 27),
                        intakeFar
                ))
                .setTangentHeadingInterpolation()
                .build();

        scoreThirdSet = follower.pathBuilder()
                .addPath(new BezierCurve(
                        intakeFar,
                        new Pose(85.622, 26.027),
                        scorePose
                ))
                .setLinearHeadingInterpolation(intakeFar.getHeading(), scorePose.getHeading())
                .build();

        park = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, parkPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading())
                .build();
    }

    private State[] stateBuilder() {
        ArrayList<State> temp = new ArrayList<>();

        temp.add(
                new State("PRELOAD")
                        .setEntry(() -> {
                            follower.followPath(scorePreload);
                            curState = 0;
                        })
                        .addTransition(new Transition(() -> !follower.isBusy(), "SCORE"))
        );

        temp.add(
                new State("INTAKE_FIRST")
                        .setEntry(() -> {
                            intake.setPower(1);
                            follower.followPath(intakeFirstSet);
                            curState = 1;
                        })
                        .setDuring(() -> {
                            if (follower.getPose().getHeading() < Math.toRadians(-75)) {
                                intake.setPower(0);
                            }
                        })
                        .addTransition(new Transition(() -> !follower.isBusy(), () -> nextPath(curState)))
        );

        temp.add(
                new State("SCORE_FIRST")
                        .setEntry(() -> {
                            follower.followPath(scoreFirstSet);
                            curState = 2;
                        })
                        .addTransition(new Transition(() -> !follower.isBusy(), "SCORE"))
        );

        temp.add(
                new State("INTAKE_SECOND")
                        .setEntry(() -> {
                            intake.setPower(1);
                            follower.followPath(intakeSecondSet);
                            curState = 3;
                        })
                        .setExit(() -> {
                                intake.setPower(0);
                        })
                        .addTransition(new Transition(() -> !follower.isBusy(), () -> nextPath(curState)))
        );

        temp.add(
                new State("SCORE_SECOND")
                        .setEntry(() -> {
                            follower.followPath(scoreSecondSet);
                            curState = 4;
                        })
                        .addTransition(new Transition(() -> !follower.isBusy(), "SCORE"))
        );

        temp.add(
                new State("INTAKE_THIRD")
                        .setEntry(() -> {
                            intake.setPower(1);
                            follower.followPath(intakeThirdSet);
                            curState = 5;
                        })
                        .setExit(() -> {
                            intake.setPower(0);
                        })
                        .addTransition(new Transition(() -> !follower.isBusy(), () -> nextPath(curState)))
        );

        temp.add(
                new State("SCORE_THIRD")
                        .setEntry(() -> {
                            follower.followPath(scoreThirdSet);
                            curState = 6;
                        })
                        .addTransition(new Transition(() -> !follower.isBusy(), "SCORE"))
        );

        temp.add(
                new State("PARK")
                        .setEntry(() -> {
                            follower.followPath(park);
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
                        })
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
            case 1: return "SCORE_FIRST";
            case 2: return "INTAKE_SECOND";
            case 3: return "SCORE_SECOND";
            case 4: return "INTAKE_THIRD";
            case 5: return "SCORE_THIRD";
            case 6: return "PARK";
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
        hood.setPosition(0.5);

        table = new InterpLUT();
        table.addPoint(24, 900, 0.144);
        table.addPoint(55, 1200, 0.64);
        table.addPoint(76, 1300, 0.66);
        table.addPoint(79, 1200, 0.54);
        table.addPoint(103, 1300, .692);
        table.addPoint(130, 1500, .718);
        table.addPoint(140, 1700, .71);

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        state = new StateMachine(stateBuilder());
        state.start();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

        curPos = follower.getPose();
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
        curPos = follower.getPose();
        double deltaX = Math.abs(curPos.getX() - goal.getX());
        double deltaY = Math.abs(curPos.getY() - goal.getY());
        curDistance = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));

        ShooterPair settings = table.lookup(curDistance);
        if (settings != null) {
            flywheel.setTargetVel(settings.getVel());
            hood.setPosition(settings.getHood());
        }

        telemetry.addData("state", state.currentState());
        telemetry.addData("robot pose", follower.getPose().toString());
        telemetry.addData("transfer", transfer.toString());
        telemetry.addData("Target Vel", flywheel.getTargetVel());
        telemetry.addData("Current Vel", flywheel.getCurrentVel());
        telemetry.addData("At Velocity?", flywheel.atVelocity());

        telemetry.update();
    }
}
