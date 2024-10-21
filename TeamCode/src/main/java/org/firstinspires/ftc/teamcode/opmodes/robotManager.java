package org.firstinspires.ftc.teamcode.opmodes;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


public class robotManager  {

    //declaring robot motors and servos
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor extensionMotor;
    private DcMotor leftLiftMotor, rightLiftMotor;

    private Servo clawGrippingServo, clawTiltServo;
    private CRServo clawWristServo;
    private CRServo leftIntakeAdjustServo, rightIntakeAdjustServo, mainIntakeServo;
    private Servo leftIntakeTiltServo, rightIntakeTiltServo;
    private Servo leftArmTiltServo, rightArmTiltServo;
    private Servo linkageExtensionServo;

    //declaring robot sensors
    private HuskyLens huskyLens;

    private IMU imu;

    private ColorSensor intakeColorSensor;


    private TouchSensor horizontalLimitSwitch, verticalLimitSwitch;

    //create variables
    boolean holdingSample = false;
    boolean holdingSpecimen = false;
    boolean sampleInIntake = false;
    boolean clawInPosition = false;
    int clawLocation = 0;

    //declare telemetry
    Telemetry roboTelemetry;

    public robotManager() {}

    public void InitializeRobot(HardwareMap robotMap, Telemetry newTelemetry) {

        //set drive motors
        frontLeft = robotMap.dcMotor.get("frontLeft");
        frontRight = robotMap.dcMotor.get("frontRight");
        backLeft = robotMap.dcMotor.get("backLeft");
        backRight = robotMap.dcMotor.get("backRight");

        //set drive motor directions
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        //set drive motor zero power behavior to brake.
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //set horizontal motors and servos
        extensionMotor = robotMap.dcMotor.get("extension");
        mainIntakeServo = robotMap.crservo.get("mainIntake");
        leftIntakeTiltServo = robotMap.servo.get("leftIntakeTilt");
        rightIntakeTiltServo = robotMap.servo.get("rightIntakeTilt");

        //set vertical motors and servos
        rightLiftMotor = robotMap.dcMotor.get("rightLift");
        leftLiftMotor = robotMap.dcMotor.get("leftLift");

        //set claw arm servos
        clawGrippingServo = robotMap.servo.get("clawGrip");
        leftArmTiltServo = robotMap.servo.get("leftArmTilt");
        rightArmTiltServo = robotMap.servo.get("rightArmTilt");
        linkageExtensionServo = robotMap.servo.get("armExtension");

        //set color sensor
        intakeColorSensor = robotMap.get(ColorSensor.class, "intakeColor");

        //setup color sensor
        if (intakeColorSensor instanceof SwitchableLight) {
            ((SwitchableLight)intakeColorSensor).enableLight(true);
        }

        //set limit switches
        horizontalLimitSwitch = robotMap.touchSensor.get("horizontalSwitch");
        verticalLimitSwitch = robotMap.touchSensor.get("verticalSwitch");

        //set IMU
        imu = robotMap.get(IMU.class, "imu");

        //setup IMU
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.LEFT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();



        //set up and update telemetry
        roboTelemetry = newTelemetry;
        roboTelemetry.addData("Status", "Robot initialized");
        roboTelemetry.update();
    }

    public void MoveRobot(double forwardsInput, double sidewaysInput, double steeringInput, double speed) {
        /*
        1) Takes forwards and sideways input and converts them into a vector.
        2) Subtracts the robot's orientation from the directions so that
           the vector is rotated to account for robot rotation.
        3) Takes the magnitude from the old components
        4) Creates new forwards and sideways components for new adjusted vector
         */


        //gets vector direction
        double direction = Math.atan2(sidewaysInput, forwardsInput);
        //subtracts the robot's orientation from direction (converted to radians)
        direction -= (-imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        //gets vector magnitude
        double magnitude = Math.sqrt(forwardsInput * forwardsInput + sidewaysInput * sidewaysInput);

        //recalculates forwards and sideways values
        forwardsInput = Math.cos(direction) * magnitude * speed;
        sidewaysInput = Math.sin(direction) * magnitude * speed;

        /*
        1) Calculates power output for each motor
        2) Clamps the power so that no motor is overloaded
        3) Sets motor power
         */

        //calculate motor power
        double leftFrontPower  = forwardsInput + sidewaysInput + (steeringInput * speed);
        double rightFrontPower = forwardsInput - sidewaysInput - (steeringInput * speed);
        double leftBackPower   = forwardsInput - sidewaysInput + (steeringInput * speed);
        double rightBackPower  = forwardsInput + sidewaysInput - (steeringInput * speed);

        //clamp max power
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower  = leftFrontPower / max;
            rightFrontPower = rightFrontPower / max;
            leftBackPower   = leftBackPower / max;
            rightBackPower  = rightBackPower / max;
        }


        //set motor power

        frontLeft.setPower(leftFrontPower);
        frontRight.setPower(rightFrontPower);
        backLeft.setPower(leftBackPower);
        backRight.setPower(rightBackPower);

        //update telemetry
        roboTelemetry.addData("Drive:", "Moving");
        roboTelemetry.update();


    }

    public void SetMotors(double frontRightPower, double frontLeftPower, double backRightPower, double backLeftPower) {
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);

        //update telemetry
        roboTelemetry.addData("Drive:", "Motor power set to: " + frontRightPower + ", " + frontLeftPower + ", " + backRightPower + ", " + backLeftPower);
        roboTelemetry.update();
    }

    public void SetIntake(double power) {
        if (IsSampleInIntake()) {
            mainIntakeServo.setPower(0.0);

            //update telemetry
            roboTelemetry.addData("Intake:", "Picked up sample");
            roboTelemetry.update();
        }
        else {
            mainIntakeServo.setPower(power);

            //update telemetry
            roboTelemetry.addData("Intake:", "Intaking at: " + power);
            roboTelemetry.update();
        }
    }



    public void ExtendHorizontal() {
        //checks if the slides are all the way out, if not, keep sliding
        boolean limitSwitch = true; //temporary until limit switch code
        if (limitSwitch) {
            extensionMotor.setPower(1.0);
        } else {
            extensionMotor.setPower(0.0);
        }
    }

    public void RetractHorizontal() {
        /*
        1 - Intake up
        2 - Retract the horizontal slides
        3 - Position claw
         */

        //checks if the intake is up
        if (leftIntakeTiltServo.getPosition() != 0.0 || rightIntakeTiltServo.getPosition() != 0.0) {
            IntakeUp();
        }
        else { //if the intake is up
            //checks if the slides are all the way in, if not, keep sliding
            if (horizontalLimitSwitch.isPressed()) {
                extensionMotor.setPower(0.0);
            } else {
                extensionMotor.setPower(-1.0);
            }

            //position the claw
            MoveClawToPosition(5);
        }
    }

    public void IntakeDown() {
        leftIntakeTiltServo.setPosition(0.2);
        rightIntakeTiltServo.setPosition(0.2);

        //update telemetry
        roboTelemetry.addData("Intake position:", "Down");
        roboTelemetry.update();
    }

    public void IntakeUp() {
        leftIntakeTiltServo.setPosition(0.0);
        rightIntakeTiltServo.setPosition(0.0);

        //update telemetry
        roboTelemetry.addData("Intake position:", "Up");
        roboTelemetry.update();
    }

    /*
    Claw positions
    0 - Unpowered
    1 - Bucket depo
    2 - Claw depo
    3 - Wall grab
    4 - Human depo
    5 - Tray

        \  1.
         \
          \
           \
   3.------**------- 2.     Front--->
          /{}\
         / {} \
        /  {}  \
    4. /   {}   \ 5.
            0.

     */

    public void MoveClawToPosition(int locationNum) {
        if (locationNum == 1) {
            SetSlidesToPosition(0.3);
            SetArmToRotation(1);
            linkageExtensionServo.setPosition(1);
            CloseClaw();
            if (IsClawInPosition(0.3, 1, 1)) {
                clawInPosition = true;
                clawLocation = locationNum;
            } else {
                clawInPosition = false;
            }
        }
        else if (locationNum == 2) {
            SetSlidesToPosition(0.3);
            SetArmToRotation(1);
            linkageExtensionServo.setPosition(1);
            CloseClaw();
            if (IsClawInPosition(0.3, 1, 1)) {
                clawInPosition = true;
                clawLocation = locationNum;
            } else {
                clawInPosition = false;
            }
        }
        else if (locationNum == 3) {
            SetSlidesToPosition(0.3);
            SetArmToRotation(1);
            linkageExtensionServo.setPosition(1);
            OpenClaw();
            if (IsClawInPosition(0.3, 1, 1)) {
                clawInPosition = true;
                clawLocation = locationNum;
            } else {
                clawInPosition = false;
            }
        }
        else if (locationNum == 4) {
            SetSlidesToPosition(0.3);
            SetArmToRotation(1);
            linkageExtensionServo.setPosition(1);
            CloseClaw();
            if (IsClawInPosition(0.3, 1, 1)) {
                clawInPosition = true;
                clawLocation = locationNum;
            } else {
                clawInPosition = false;
            }
        }
        else if (locationNum == 5) {
            SetSlidesToPosition(0.3);
            SetArmToRotation(1);
            linkageExtensionServo.setPosition(1);
            OpenClaw();
            if (IsClawInPosition(0.3, 1, 1)) {
                clawInPosition = true;
                clawLocation = locationNum;
            } else {
                clawInPosition = false;
            }
        }


    }

    private boolean IsClawInPosition(double slidePosition, double armRotation, double linkageExtension) {
        int ticksToFullExtension = 100;
        int ticks = (int)(ticksToFullExtension * slidePosition);

        if (Math.abs(leftArmTiltServo.getPosition() - armRotation) < 0.1 &&
                Math.abs(rightArmTiltServo.getPosition() - armRotation) < 0.1 &&
                Math.abs(linkageExtensionServo.getPosition() - linkageExtension) < 0.1 &&
                Math.abs(leftLiftMotor.getCurrentPosition() - ticks) < 0.1 &&
                Math.abs(rightLiftMotor.getCurrentPosition() - ticks) < 0.1) {

            return true;
        }
        return false;
    }

    private void SetArmToRotation(double position) {
        leftArmTiltServo.setPosition(position);
        rightArmTiltServo.setPosition(position);
    }

    //a value between 0.0 and 1.0, where 0.0 is full retraction and 1.0 is full extension
    private void SetSlidesToPosition(double position) {
        int ticksToFullExtension = 100;
        int ticks = (int)(ticksToFullExtension * position);

        rightLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLiftMotor.setPower(0.4);
        rightLiftMotor.setTargetPosition(ticks);
        rightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLiftMotor.setPower(0.4);
        leftLiftMotor.setTargetPosition(ticks);
        leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if(rightLiftMotor.isBusy() || leftLiftMotor.isBusy()){
            telemetry.addData("Vertical slides:","Moving to" + ticks);
        }
        else{
            rightLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightLiftMotor.setPower(0);
            leftLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftLiftMotor.setPower(0);

            if (position == 0) {
                rightLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
        }
    }

    public void OpenClaw() {
        if (clawInPosition) {
            clawGrippingServo.setPosition(0);

            //update telemetry
            roboTelemetry.addData("Claw:", "Open");
            roboTelemetry.update();
        }
    }

    public void CloseClaw() {
        if (clawInPosition) {
            clawGrippingServo.setPosition(90);

            //update telemetry
            roboTelemetry.addData("Claw:", "Closed");
            roboTelemetry.update();
        }
    }

    public boolean IsSampleInIntake() {
        //gets color, converts it to HSV
        int redColor = 1500;
         if(intakeColorSensor.red() > 1300 ){
             return true;
         }
         return false;
    }

    /*
    Prebuilt functions:
    - Retract horizontal extension
    - Stop in-taking when there is a sample
    - Grab sample from tray
    - Move claw to clip-on
    - Move claw to bucket
    - Move claw to human zone

    Mechanism focused:
        Controller 1 Layout (Driver):
        L-Stick - Move
        R-Stick - Rotate
        RB - Intake
        LB - Outtake
        X - First bar hang
        Y - Second bar hang

        Controller 2 Layout (Attachments):
        Y - Position claw for bucket
        B - Position claw for clip hang
        A - Position claw to drop in human zone\
        X - Position claw to grab from wall
        RB - Close claw
        LB - Open claw
        LT - Retract horizontal (intake up)
        RT - Extend horizontal
        DpadDown - Intake down




     */
}



