package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Outtake.Shooter;

@TeleOp(name = "run ts", group = ".")
public class TeleOpMAIN extends LinearOpMode {
    private Follower drive;
    private Pose pose;
    public static Pose target = new Pose(-52, 52, Math.toRadians(135));

    /* =======================
       SHOOTER HARDWARE
       ======================= */
    private DcMotorEx shooterLeft;
    private DcMotorEx shooterRight;
    private DcMotorEx intake;
    private DcMotorEx intake2;
    private Shooter shoot;
    private Servo hood;

    /* =======================
       FSM HARDWARE
       ======================= */
    private Servo elevatorLeft;
    private Servo elevatorRight;
    private Servo spindexer;

    /* =======================
       CONSTANTS (INLINE)
       ======================= */
    private static final double ELEVATOR_LEFT_UP = 214.0 / 355;
    private static final double ELEVATOR_LEFT_DOWN = 324.0 / 355;
    private static final double ELEVATOR_RIGHT_UP = 205.0 / 355;
    private static final double ELEVATOR_RIGHT_DOWN = 324.0 / 355;

    private static final double SPINDEXER_ONE = 0.0 / 355;
    private static final double SPINDEXER_TWO = 140.0 / 355;
    private static final double SPINDEXER_THREE = 265.0 / 355;

    /* =======================
       FSM STATE
       ======================= */
    private enum ShooterState {
        IDLE,
        SPINDEXER_SET,
        RIGHT_DOWN,
        LEFT_DOWN,
        ELEVATORS_UP,
        RESET_TO_ONE
    }

    private ShooterState shooterState = ShooterState.IDLE;
    private boolean shooterActive = false;
    private int shooterBall = 1;
    private final ElapsedTime shooterTimer = new ElapsedTime();

    @Override
    public void runOpMode() {

        /* =======================
           HARDWARE INIT
           ======================= */
        shooterLeft = hardwareMap.get(DcMotorEx.class, "sl");
        shooterRight = hardwareMap.get(DcMotorEx.class, "sr");
        shoot = new Shooter(hardwareMap);
        intake = hardwareMap.get(DcMotorEx.class, "li");
        intake2 = hardwareMap.get(DcMotorEx.class, "ri");
        hood = hardwareMap.get(Servo.class, "hood");

        elevatorLeft = hardwareMap.get(Servo.class, "el");
        elevatorRight = hardwareMap.get(Servo.class, "er");
        spindexer = hardwareMap.get(Servo.class, "spin");

        shooterLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooterRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        drive = Constants.createFollower(hardwareMap);
        drive.setStartingPose(new Pose(0, 0, 0));
        elevatorLeft.setPosition(ELEVATOR_LEFT_UP);
        elevatorRight.setPosition(ELEVATOR_RIGHT_UP);

        waitForStart();

        drive.startTeleOpDrive(true);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        while (opModeIsActive()) {

            shoot.update();

            /* =======================
               DRIVE
               ======================= */
            drive.update();
            drive.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    true
            );

            /* =======================
               CIRCLE → START FSM
               ======================= */
            if (gamepad1.circleWasPressed() && !shooterActive) {
                shooterActive = true;
                shooterBall = 1;
                shooterState = ShooterState.SPINDEXER_SET;
                shooterTimer.reset();
            }

            /* =======================
               FSM UPDATE
               ======================= */
            if (shooterActive) {
                switch (shooterState) {

                    case SPINDEXER_SET:
                        if (shooterBall == 1) spindexer.setPosition(SPINDEXER_ONE);
                        if (shooterBall == 2) spindexer.setPosition(SPINDEXER_TWO);
                        if (shooterBall == 3) spindexer.setPosition(SPINDEXER_THREE);

                        if (shooterTimer.milliseconds() > 250 && shoot.atVelocity()) {
                            shooterState = ShooterState.RIGHT_DOWN;
                            shooterTimer.reset();
                        }
                        break;

                    case RIGHT_DOWN:
                        elevatorRight.setPosition(ELEVATOR_RIGHT_DOWN);
                        if (shooterTimer.milliseconds() > 120) {
                            shooterState = ShooterState.LEFT_DOWN;
                            shooterTimer.reset();
                        }
                        break;

                    case LEFT_DOWN:
                        elevatorLeft.setPosition(ELEVATOR_LEFT_DOWN);
                        if (shooterTimer.milliseconds() > 400) {
                            shooterState = ShooterState.ELEVATORS_UP;
                            shooterTimer.reset();
                        }
                        break;

                    case ELEVATORS_UP:
                        elevatorLeft.setPosition(ELEVATOR_LEFT_UP);
                        elevatorRight.setPosition(ELEVATOR_RIGHT_UP);

                        if (shooterTimer.milliseconds() > 350) {
                            if (shooterBall < 3) {
                                shooterBall++;
                                shooterState = ShooterState.SPINDEXER_SET;
                            } else {
                                shooterState = ShooterState.RESET_TO_ONE;
                            }
                            shooterTimer.reset();
                        }
                        break;

                    case RESET_TO_ONE:
                        spindexer.setPosition(SPINDEXER_ONE);
                        shooterActive = false;
                        shooterState = ShooterState.IDLE;
                        break;
                }
            }
            if (gamepad1.leftBumperWasPressed()) {
                intake.setPower(0);
                intake2.setPower(0);
            }

            if (gamepad1.rightBumperWasPressed()) {
                intake.setPower(1);
                intake2.setPower(1);
            }
            if (gamepad1.right_trigger > 0.1) {
                intake.setPower(-1);
                intake2.setPower(-1);
            }

            if (gamepad1.optionsWasPressed()) {
                drive.setPose(new Pose(129.5, 129.5, Math.toRadians(38)));
            } else if (gamepad1.shareWasPressed()) {
                drive.setPose(new Pose(21.5, 129.5, Math.toRadians(143)));
            }


            /* =======================
               DPAD SHOOTER PRESETS
               ======================= */
            if (gamepad1.dpadDownWasPressed()) {
                shoot.setTargetVel(1400);
                hood.setPosition(0.542);
            }

            if (gamepad1.dpadLeftWasPressed()) {
                shoot.setTargetVel(1000);
                hood.setPosition(0.24);
            }

            if (gamepad1.dpadRightWasPressed()) {
                shoot.setTargetVel(1100);
                hood.setPosition(0.28);
            }

            if (gamepad1.dpadUpWasPressed()) {
                shoot.setTargetVel(0);
                hood.setPosition(0.5);
            }
        }
    }
}
