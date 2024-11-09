package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class TurnTable {
    private Servo bottomServo;

    public TurnTable(HardwareMap hardwareMap) {
        bottomServo = hardwareMap.get(Servo.class, "bottomServo");
    }
}
