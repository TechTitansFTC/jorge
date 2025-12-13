package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.Outtake.Shooter;

@Autonomous(name = "LeaveZoneAuto", group = "Auto")
public class LeaveZoneAuto extends LinearOpMode {
    private Shooter shoot;
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

        // ===== DRIVETRAIN =====
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("fl");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("bl");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("fr");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("br");

        // ===== SHOOTER + HOOD =====
        DcMotor shooterLeft = hardwareMap.dcMotor.get("sl");
        DcMotor shooterRight = hardwareMap.dcMotor.get("sr");
        Servo hood = hardwareMap.servo.get("hood");
        shoot = new Shooter(hardwareMap);

        // ===== INTAKE (optional) =====
        DcMotor intake = hardwareMap.dcMotor.get("li");
        DcMotor intake2 = hardwareMap.dcMotor.get("ri");
        intake2.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        if (isStopRequested()) return;

        // ===== MOVE FORWARD TO LEAVE STARTING ZONE =====
        drive(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor,
                0.5, 0.5, 0.5, 0.5, 100);

        // ===== SHOOT 3 BALLS =====
        shoot.setTargetVel(1400);
        hood.setPosition(0.542);



        for (int ball = 1; ball <= 3; ball++) {
            // Set spindexer/elevator if you have them (optional)
            // Here we just simulate shooting 3 balls with timing
            sleep(10000);// time to shoot one ball
        }

        shooterLeft.setPower(0);
        shooterRight.setPower(0);

        // ===== STOP EVERYTHING =====
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
    }
}
