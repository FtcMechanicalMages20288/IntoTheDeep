package org.firstinspires.ftc.teamcode.testing;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "intakeTestNew", group = "intake")
public class newIntakeTest extends OpMode {

    private CRServo rightFlipper, leftFlipper, leftFlipperB;
    @Override
    public void init() {
        rightFlipper = hardwareMap.get(CRServo.class, "extenddown");
        leftFlipper = hardwareMap.get(CRServo.class,"servointake");
        leftFlipperB = hardwareMap.get(CRServo.class,"servointake1");
    }

    @Override
    public void loop() {
        rightFlipper.setPower(0.4);
        leftFlipper.setPower(-0.4);
        leftFlipperB.setPower(-0.4);

    }
}
