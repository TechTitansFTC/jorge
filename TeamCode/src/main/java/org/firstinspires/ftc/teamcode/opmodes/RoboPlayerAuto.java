package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Outtake.Shooter;

@Autonomous(name = "RoboPlayersAuto", group = "Auto")
public class RoboPlayerAuto extends LinearOpMode {
    private Shooter shoot;
    private enum FSMState {
        SPINDEXER_SET,
        RIGHT_DOWN,
        LEFT_DOWN,
        ELEVATORS_UP,
        RESET_TO_ONE,
        IDLE
    }


    // ===== MOVEMENT HELPER =====
    private void drive(DcMotor fl, DcMotor bl, DcMotor fr, DcMotor br,
                       double flp, double blp, double frp, double brp, long timeMs) {
        fl.setPower(flp);
        bl.setPower(blp);
        fr.setPower(frp);
        br.setPower(brp);
        sleep(timeMs);
        fl.setPower(0);
        bl.setPower(0);
        fr.setPower(0);
        br.setPower(0);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("fl");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("bl");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("fr");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("br");

        DcMotor shooterLeft = hardwareMap.dcMotor.get("sl");
        DcMotor shooterRight = hardwareMap.dcMotor.get("sr");
        Servo hood = hardwareMap.servo.get("hood");
        shoot = new Shooter(hardwareMap);

        Servo elevatorLeft = hardwareMap.get(Servo.class, "el");
        Servo elevatorRight = hardwareMap.get(Servo.class, "er");
        Servo spindexer = hardwareMap.get(Servo.class, "spin");

        DcMotor intake = hardwareMap.dcMotor.get("li");
        DcMotor intake2 = hardwareMap.dcMotor.get("ri");
        intake2.setDirection(DcMotorSimple.Direction.REVERSE);

        final double ELEVATOR_LEFT_UP = 214.0 / 355;
        final double ELEVATOR_LEFT_DOWN = 324.0 / 355;
        final double ELEVATOR_RIGHT_UP = 205.0 / 355;
        final double ELEVATOR_RIGHT_DOWN = 324.0 / 355;

        final double SPINDEXER_ONE = 0.0 / 355;
        final double SPINDEXER_TWO = 140.0 / 355;
        final double SPINDEXER_THREE = 265.0 / 355;


        FSMState fsmState = FSMState.SPINDEXER_SET;
        int fsmBall = 1;
        boolean fsmActive = true;
        ElapsedTime fsmTimer = new ElapsedTime();

        waitForStart();
        if (isStopRequested()) return;

        drive(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor,
                -0.5, -0.5, -0.5, -0.5, 300);

        shoot.setTargetVel(1000);
        hood.setPosition(0.24);

        while (fsmActive && opModeIsActive()) {

            switch (fsmState) {

                case SPINDEXER_SET:
                    if (fsmBall == 1) spindexer.setPosition(SPINDEXER_ONE);
                    if (fsmBall == 2) spindexer.setPosition(SPINDEXER_TWO);
                    if (fsmBall == 3) spindexer.setPosition(SPINDEXER_THREE);

                    if (fsmTimer.milliseconds() > 250) {
                        fsmState = FSMState.RIGHT_DOWN;
                        fsmTimer.reset();
                    }
                    break;

                case RIGHT_DOWN:
                    elevatorRight.setPosition(ELEVATOR_RIGHT_DOWN);
                    if (fsmTimer.milliseconds() > 120) {
                        fsmState = FSMState.LEFT_DOWN;
                        fsmTimer.reset();
                    }
                    break;

                case LEFT_DOWN:
                    elevatorLeft.setPosition(ELEVATOR_LEFT_DOWN);
                    if (fsmTimer.milliseconds() > 400) {
                        fsmState = FSMState.ELEVATORS_UP;
                        fsmTimer.reset();
                    }
                    break;

                case ELEVATORS_UP:
                    elevatorLeft.setPosition(ELEVATOR_LEFT_UP);
                    elevatorRight.setPosition(ELEVATOR_RIGHT_UP);

                    if (fsmTimer.milliseconds() > 350) {
                        if (fsmBall < 3) {
                            fsmBall++;
                            fsmState = FSMState.SPINDEXER_SET;
                            fsmTimer.reset();
                        } else {
                            fsmState = FSMState.RESET_TO_ONE;
                            fsmTimer.reset();
                        }
                    }
                    break;

                case RESET_TO_ONE:
                    spindexer.setPosition(SPINDEXER_ONE);
                    shoot.setTargetVel(0);
                    fsmActive = false;
                    fsmState = FSMState.IDLE;
                    break;

                default:
                    break;
            }

            idle();
        }

        wait(300);
        shooterLeft.setPower(0);
        shooterRight.setPower(0);
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
    }
}
