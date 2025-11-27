package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.templates.PIDF_Tuner;
import org.firstinspires.ftc.teamcode.util.wrappers.nCRServo;

@TeleOp(name = "turret tuning", group = "tuners")
public class turretPID_tuner extends PIDF_Tuner {
    @Override
    public void init() {
        super.init();
        nCRServo turret = new nCRServo("spin", "sr", hardwareMap);
        setHardware(turret);
    }
}
