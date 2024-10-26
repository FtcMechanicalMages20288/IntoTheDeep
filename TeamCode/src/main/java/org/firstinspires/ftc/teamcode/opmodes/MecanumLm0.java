package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name = "MecanumlmO", group = "TeleOp")

public class MecanumLm0 extends LinearOpMode {


    //DriveTrain Motors
    private DcMotor extendcentral, leftvert, rightvert;

    private TouchSensor vertswitch, hortswitch;


    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backRight;
    DcMotor backLeft;

    double left_drivePower;
    double right_drivePower;
    double back_right_drivePower;
    double back_left_drivePower;




    private Servo armExtend, extenddown, claw, clawrotate;
    private CRServo servointake, servointake1, leftrotate, rightrotate;

    private IMU imu;





    @Override
    public void runOpMode() throws InterruptedException {

        double x = 0.32;

        if (opModeInInit()) {
            initCode();
            claw.setPosition(0.7);
            clawrotate.setPosition(0);
          //  armExtend.setPosition(x);

        }

        waitForStart();

        while (opModeIsActive()) {

         movementController();
         if(gamepad2.y){
             armExtend.setPosition(x);
         }

        /*
        1) Takes forwards and sideways input and converts them into a vector.
        2) Subtracts the robot's orientation from the directions so that
           the vector is rotated to account for robot rotation.
        3) Takes the magnitude from the old components
        4) Creates new forwards and sideways components for new adjusted vector


            double forwardsInput = -gamepad1.left_stick_y;
            double sidewaysInput = gamepad1.left_stick_x;
            double steeringInput = gamepad1.right_stick_x;
            double speed = 1;

            //gets vector direction
            double direction = Math.atan2(sidewaysInput, forwardsInput);
            //subtracts the robot's orientation from direction (converted to radians)
            direction -= (-imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

            //gets vector magnitude
            double magnitude = Math.sqrt(forwardsInput * forwardsInput + sidewaysInput * sidewaysInput);

            //recalculates forwards and sideways values
            forwardsInput = Math.cos(direction) * magnitude * speed;
            sidewaysInput = Math.sin(direction) * magnitude * speed;


        1) Calculates power output for each motor
        2) Clamps the power so that no motor is overloaded
        3) Sets motor power


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

                 */






               if (gamepad2.dpad_left) {
                   leftrotate.setPower(1);
                   rightrotate.setPower(1);
                   armExtend.setPosition(x);
                   clawrotate.setPosition(0);


               } else if (gamepad2.dpad_right) {
                   clawrotate.setPosition(0.5);
                   leftrotate.setPower(-1);
                   rightrotate.setPower(-1);

               } else {
                   leftrotate.setPower(-0.1);
                   rightrotate.setPower(-0.1);
               }


            if (gamepad1.a) {
                imu.resetYaw();
            }


            boolean ExtendForward = gamepad2.right_trigger > 0.3;
            boolean ExtendDown = gamepad2.left_trigger > 0.3;


            if (ExtendForward) {
                extendcentral.setPower(1);
            }

            else {

                extendcentral.setPower(0);
            }


            if (ExtendDown) {
                extendcentral.setPower(-0.85);
/*
                if (hortswitch.isPressed()) {
                    extendcentral.setPower(-0.1);
                    telemetry.addData("Horizontal Slides", "Ready");
                    gamepad2.rumble(1000);

                } else {
                    extendcentral.setPower(-0.4);
                    extendcentral.setPower(-0.4);
                    telemetry.addData("Horizontal Slides", "Not Ready");
                }

 */
            }

            extendcentral.setPower(0);

            double extendpos = 0.4;
            if (gamepad2.a && !ExtendForward) {
                extenddown.setPosition(extendpos+0.05);
                servointake.setPower(-1);
                servointake1.setPower(1);

            }
            else if (gamepad2.b && !ExtendForward) {
                extenddown.setPosition(extendpos);
                servointake.setPower(-1);
                servointake1.setPower(-1);

            }
            else if (gamepad2.a && ExtendForward) {
                extenddown.setPosition(extendpos+0.05);
                servointake.setPower(1);
                servointake1.setPower(1);
                extendcentral.setPower(1);

            }
            else{
                servointake.setPower(0);
                servointake1.setPower(0);
                extenddown.setPosition(0.75);
            }






            if (gamepad2.x) {
                claw.setPosition(1);
                armExtend.setPosition(0.6);

            }
            if(gamepad2.y){
                claw.setPosition(0.7);
                armExtend.setPosition(0.6);


            }

            //close


            //open






                    /*
                    NormalizedColorSensor blocksensor= hardwareMap.get(NormalizedColorSensor.class, "blocksensor");
                    DistanceSensor distanceblocksensor = (DistanceSensor) blocksensor;

                    if(distanceblocksensor.getDistance(DistanceUnit.CM) < 2){
                        servointake.setPower(0);
                        sleep(1200);
                        servointake.setPower(-1);
                    }

                } else {
                    extenddown.setPosition(0.2);
                    servointake.setPower(0);
                }

                if (gamepad2.b) {
                    extenddown.setPosition(0.3);

                }

                     */

            boolean SlidesUp = gamepad2.right_bumper;
            boolean SlidesDown = gamepad2.left_bumper;


            if (SlidesUp) {

                rightvert.setPower(0.4);
                leftvert.setPower(0.4);
            }
            rightvert.setPower(0);
            leftvert.setPower(0);


            if (SlidesDown) {

                rightvert.setPower(-0.4);
                leftvert.setPower(-0.4);
                telemetry.addData("Vertical Slides", "Not Ready");

                if (vertswitch.isPressed()) {
                    rightvert.setPower(-0.1);
                    leftvert.setPower(-0.1);
                    telemetry.addData("Vertical Slides", "Ready");
                    gamepad2.rumble(1000);

                } else {
                    rightvert.setPower(-0.4);
                    leftvert.setPower(-0.4);
                    telemetry.addData("Vertical Slides", "Not Ready");
                }

                rightvert.setPower(-0.4);
                leftvert.setPower(-0.4);

            }
            rightvert.setPower(0);
            leftvert.setPower(0);
        }


        if (isStopRequested()) {
            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);

        }
    }


    private void initCode() {

        imu = hardwareMap.get(IMU.class, "imu");

        //setup IMU
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();

        frontLeft = hardwareMap.dcMotor.get("lm");

        frontRight = hardwareMap.dcMotor.get("rm");
        backRight = hardwareMap.dcMotor.get("brm");
        backLeft = hardwareMap.dcMotor.get("blm");

        extendcentral = hardwareMap.dcMotor.get("extendcentral");

        rightvert = hardwareMap.dcMotor.get("rightvert");
        leftvert = hardwareMap.dcMotor.get("leftvert");

        leftrotate = hardwareMap.get(CRServo.class, "leftrotate");
        rightrotate = hardwareMap.get(CRServo.class, "rightrotate");

        armExtend = hardwareMap.get(Servo.class, "armExtend");
        claw = hardwareMap.get(Servo.class, "claw");
        clawrotate = hardwareMap.get(Servo.class, "clawrotate");
        servointake = hardwareMap.get(CRServo.class, "servointake");
        servointake1 = hardwareMap.get(CRServo.class, "servointake1");

        extenddown = hardwareMap.get(Servo.class, "extenddown");

        //Sensors
        vertswitch = hardwareMap.get(TouchSensor.class, "vertswitch");
        hortswitch = hardwareMap.get(TouchSensor.class, "hortswitch");



        //MOTOR DIRECTION SWITCHING
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);


        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftrotate.setDirection(CRServo.Direction.REVERSE);




    }
    private void movementController( ){
        right_drivePower = gamepad1.right_stick_y;
        back_left_drivePower = gamepad1.left_stick_y;
        left_drivePower = gamepad1.left_stick_y;
        back_right_drivePower = gamepad1.right_stick_y;


        frontLeft.setPower(left_drivePower);
        frontRight.setPower(right_drivePower);
        backLeft.setPower(left_drivePower);
        backRight.setPower(right_drivePower);


        boolean rightbumper = gamepad1.right_bumper; //Strafe Right
        boolean leftbumper = gamepad1.left_bumper; //Strafe Left

        if(rightbumper){
            frontLeft.setPower(1);
            frontRight.setPower(-1);
            backRight.setPower(1);
            backLeft.setPower(-1);
        }
        else if(leftbumper){
            frontLeft.setPower(-1);
            frontRight.setPower(1);
            backRight.setPower(-1);
            backLeft.setPower(1);
        }
    }
}

//Methods