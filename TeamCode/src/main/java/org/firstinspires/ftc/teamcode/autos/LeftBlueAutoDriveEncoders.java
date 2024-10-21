package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="LeftBlueAutoDriveEncoders", group = "auto")
public class LeftBlueAutoDriveEncoders extends LinearOpMode {

    static final int MOTOR_TICK_COUNTS = 751;
    static final double WHEEL_DIAMETER_INCHES = 3.70;
    static final double CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER_INCHES;

    private DcMotor right_drive, left_drive, back_right_drive, back_left_drive, rightvert, leftvert;
    private Servo leftrotate, rightrotate, armExtend, extenddown;
    private CRServo servointake;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize drive motors
        left_drive = hardwareMap.dcMotor.get("lm");
        right_drive = hardwareMap.dcMotor.get("rm");
        back_right_drive = hardwareMap.dcMotor.get("brm");
        back_left_drive = hardwareMap.dcMotor.get("blm");

        //Intialize attachment motors
        rightvert = hardwareMap.dcMotor.get("rightvert");
        leftvert = hardwareMap.dcMotor.get("leftvert");

        left_drive.setDirection(DcMotor.Direction.REVERSE);
        back_left_drive.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //extenddown.setPosition(0.85);

        waitForStart();

        //Trajectories
        moveForward(20);

       // moveForward(7);

    }

    private void moveBackward(double distanceInches) {
        int targetPosition = calculateTargetPosition(distanceInches);

        setTargetPosition(targetPosition, targetPosition, targetPosition, targetPosition);
        setPower(0.3);
        runToPosition();

        stopMotors();
        resetEncoders();
    }

    // Method to move backward a given distance (in inches)
    private void moveForward(double distanceInches) {
        int targetPosition = calculateTargetPosition(-distanceInches);

        setTargetPosition(targetPosition, targetPosition, targetPosition, targetPosition);
        setPower(0.3);
        runToPosition();

        stopMotors();
        resetEncoders();


    }


    private void strafeRight(double distanceInches) {
        int targetPosition = calculateTargetPosition(distanceInches);


        setTargetPosition(-targetPosition, targetPosition, targetPosition, -targetPosition);
        setPower(0.45);
        runToPosition();

        stopMotors();
        resetEncoders();
    }


    private void strafeLeft(double distanceInches) {
        int targetPosition = calculateTargetPosition(distanceInches);

        // Reverse the other side for strafing
        setTargetPosition(targetPosition, -targetPosition, -targetPosition, targetPosition);
        setPower(0.45);
        runToPosition();

        stopMotors();
        resetEncoders();
    }

    private void turnLeft(double distanceInches) {
        int targetPosition = calculateTargetPosition(distanceInches);;


        setTargetPosition(targetPosition, -targetPosition, targetPosition, -targetPosition);
        setPower(0.3);
        runToPosition();

        stopMotors();
        resetEncoders();
    }

    private void turnRight(double distanceInches) {
        int targetPosition = calculateTargetPosition(distanceInches);;


        setTargetPosition(-targetPosition, targetPosition, -targetPosition, targetPosition);
        setPower(0.3);
        runToPosition();

        stopMotors();
        resetEncoders();
    }


    private void slideUp(double power) {
        leftvert.setPower(power);
        rightvert.setPower(power);

    }

    private void slideDown(double power) {
        leftvert.setPower(-1*power);
        rightvert.setPower(-1*power);

    }

    private int calculateTargetPosition(double distanceInches) {
        double rotationsNeeded = distanceInches / CIRCUMFERENCE;
        return (int) (rotationsNeeded * MOTOR_TICK_COUNTS);
    }


    private void setTargetPosition(int leftFront, int rightFront, int leftBack, int rightBack) {
        left_drive.setTargetPosition(leftFront);
        right_drive.setTargetPosition(rightFront);
        back_left_drive.setTargetPosition(leftBack);
        back_right_drive.setTargetPosition(rightBack);
    }

    private void setPower(double power) {
        left_drive.setPower(power);
        right_drive.setPower(power);
        back_left_drive.setPower(power);
        back_right_drive.setPower(power);
    }

    private void runToPosition() {
        left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (opModeIsActive() && left_drive.isBusy() && right_drive.isBusy() && back_left_drive.isBusy() && back_right_drive.isBusy()) {
            telemetry.addData("Encoder Right", right_drive.getCurrentPosition());
            telemetry.addData("Encoder Left:", left_drive.getCurrentPosition());
            telemetry.addData("Encoder Back Left:", back_left_drive.getCurrentPosition());
            telemetry.addData("Encoder Back Right:", back_right_drive.getCurrentPosition());
            telemetry.update();
        }
    }


    private void stopMotors() {
        left_drive.setPower(0);
        right_drive.setPower(0);
        back_right_drive.setPower(0);
        back_left_drive.setPower(0);
    }

    private void resetEncoders() {
        left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void initCode() {

    }

}
