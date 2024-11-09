package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class SwingingArm {
    private Servo topServo;

    public SwingingArm(HardwareMap hardwareMap) {
        topServo = hardwareMap.get(Servo.class, "topServo");
    }
}
