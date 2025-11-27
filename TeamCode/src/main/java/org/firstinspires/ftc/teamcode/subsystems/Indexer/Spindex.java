package org.firstinspires.ftc.teamcode.subsystems.Indexer;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.util.templates.Feature;
import org.firstinspires.ftc.teamcode.util.wrappers.nServo;

import java.util.ArrayList;

public class Spindex extends Feature {

    private nServo spindex;
    private Constants.Positions currentPosition;
//    private REVColorSensorV3 c, l, r;

    public Spindex (HardwareMap map) {
        super(new ArrayList<>());

        spindex = new nServo("spin", map, nServo.ServoType.AXON_MAX_V2);
//        c = new REVColorSensorV3(map.get(I2cDeviceSynch.class, "center"));
//        l = new REVColorSensorV3(map.get(I2cDeviceSynch.class, "left"));
//        r = new REVColorSensorV3(map.get(I2cDeviceSynch.class, "right"));
        currentPosition = Constants.Positions.CENTER;

        hw.add(spindex);
    }

    public void update() {
        spindex.getServo().setPosition(currentPosition.getPosition());
        spindex.update();
        //TODO: color sensor shit
    }

    public void setPosition(Constants.Positions position) {
        currentPosition = position;
    }

    public Constants.Positions getPosition() {
        return currentPosition;
    }

    public boolean inPosition() {
        return spindex.inPosition();
    }
}
