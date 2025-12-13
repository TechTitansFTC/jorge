package org.firstinspires.ftc.teamcode.testers;

//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.util.PIDF;

//@Config
@TeleOp(name = "PIDF Tuner", group = "tuners")
public class shooterPIDFTuner extends LinearOpMode {

    public static double P = 0;
    public static double D = 0;
    public static double kV = 0;

    public static double targetVel = 0;
    public static double tolerance = 0;


//    MultipleTelemetry telemetryData = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    private static final PIDF shooter = new PIDF(P, D, 0);
    private DcMotorEx leftShooter, rightShooter;

    @Override
    public void runOpMode() throws InterruptedException {
        leftShooter = hardwareMap.get(DcMotorEx.class, "sl");
        rightShooter = hardwareMap.get(DcMotorEx.class, "sr");
        VoltageSensor voltage = hardwareMap.get(VoltageSensor.class, "Control Hub");

//        leftShooter.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        if(isStopRequested()) return;

        while(opModeIsActive()) {
            double currentVel = leftShooter.getVelocity();

            targetVel = Math.max(0, targetVel);

            shooter.setP(P);
            shooter.setD(D);
            shooter.setF(kV * targetVel);

        double power = shooter.calculate(currentVel, targetVel);
        power /= voltage.getVoltage();

            leftShooter.setPower(power);
            rightShooter.setPower(power);

//            telemetryData.addData("power", power);
//            telemetryData.addData("target velocity", targetVel);
//            telemetryData.addData("actual velocity", currentVel);
//            telemetryData.update();
            shooter.setTolerance(tolerance);
        }
    }
}
