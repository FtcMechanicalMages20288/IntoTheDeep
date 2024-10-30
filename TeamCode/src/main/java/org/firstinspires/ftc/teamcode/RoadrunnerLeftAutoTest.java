package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
@Autonomous(name = "RoadrunnerCycle", group = "Autonomous")
public class RoadrunnerLeftAutoTest extends LinearOpMode {

    // Horizontal Extension
    public class Hortextension {
        private DcMotor hortextension;

        public Hortextension(HardwareMap hardwareMap) {
            hortextension = hardwareMap.get(DcMotor.class, "extendcentral");
            hortextension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            hortextension.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        public Action extensionforward() {
            return new Extensionforward();
        }

        public class Extensionforward implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    initialized = true;
                    hortextension.setPower(0.6);
                }

                NormalizedColorSensor blocksensor= hardwareMap.get(NormalizedColorSensor.class, "blocksensor");
                DistanceSensor distanceblocksensor = (DistanceSensor) blocksensor;

                double pos = hortextension.getCurrentPosition();
                packet.put("horizontal extension forward", pos);


                //pos >= 2700 && pos <= 3000 ||
                if (distanceblocksensor.getDistance(DistanceUnit.CM) < 3) {
                    hortextension.setPower(0);
                    return true;
                } else {
                    double distance = distanceblocksensor.getDistance(DistanceUnit.CM);
                    packet.put("Distance to block", distance);
                    hortextension.setPower(0.6);
                    return false;
                }
            }
        }

        public Action extensionback() {
            return new Extensionback();  // Return an instance of the Extensionback class
        }

        public class Extensionback implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    hortextension.setPower(-0.45); // Change accordingly
                    initialized = true;
                }

                TouchSensor slidesensor;
                slidesensor = hardwareMap.get(TouchSensor.class, "hortswitch");
                double pos = hortextension.getCurrentPosition();
                packet.put("horizontal extension back", pos);

                if (slidesensor.isPressed()) {
                    hortextension.setPower(0);
                    return true;
                } else {
                    hortextension.setPower(-0.45); // Change accordingly
                    return false;
                }
            }
        }
    }

    public class Lift {
        private DcMotor liftRight, liftLeft;

        public Lift(HardwareMap hardwareMap) {
           liftRight = hardwareMap.get(DcMotor.class, "rightvert");
           liftLeft = hardwareMap.get(DcMotor.class, "leftvert");

            liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            liftLeft.setDirection(DcMotorSimple.Direction.FORWARD);

            liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            liftRight.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        public class LiftUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                double liftPower = 0.5;
                if (!initialized) {
                    liftLeft.setPower(liftPower);
                    liftRight.setPower(liftPower);
                    initialized = true;
                }

                long sleepDuration = 2000; // Run for 2 seconds
                long startTime = System.currentTimeMillis(); // Record the start time



                //Check this code
                while (System.currentTimeMillis() - startTime < sleepDuration) {
                    liftRight.setPower(liftPower);
                    liftLeft.setPower(liftPower);
                }

                liftRight.setPower(0);
                liftLeft.setPower(0);
                return true;

            }
        }
        public Action liftUp() {
            return new LiftUp();
        }

        public class LiftDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    liftRight.setPower(-0.8);
                    liftLeft.setPower(-0.8);
                    initialized = true;
                }

                TouchSensor vertsensor;
                vertsensor = hardwareMap.get(TouchSensor.class, "vertsensor");

                if (vertsensor.isPressed()) {
                    liftRight.setPower(0);
                    liftLeft.setPower(0);
                    return true;

                } else {
                    liftRight.setPower(-0.4);
                    liftLeft.setPower(-0.4);
                    return false;
                }
            }
        }
        public Action liftDown(){
            return new LiftDown();
        }
    }



    // Extend Intake

    public class Intake {
        private CRServo intakeLeft, intakeRight;

        public Intake(HardwareMap hardwareMap) {
            intakeLeft = hardwareMap.get(CRServo.class, "intakeleft");
            intakeRight = hardwareMap.get(CRServo.class, "intakeright");
        }

        public class CloseIntake implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intakeLeft.setPower(0);
                intakeRight.setPower(0);
                return true; // Return true to indicate action is complete
            }
        }

        public Action closeIntake() {
            return new CloseIntake();
        }

        public class OpenIntake implements Action {
            private long startTime;
            private final long duration; // Duration for which to keep the intake open

            public OpenIntake(long duration) {
                this.duration = duration; // Duration in milliseconds
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (startTime == 0) {
                    intakeLeft.setPower(1);
                    intakeRight.setPower(-1);
                    startTime = System.currentTimeMillis(); // Initialize start time
                }

                // Check if the duration has elapsed
                if (System.currentTimeMillis() - startTime >= duration) {
                    intakeLeft.setPower(0);
                    intakeRight.setPower(0);
                    return true; // Action complete
                }
                return false; // Still running
            }
        }

        public Action openIntake(long duration) {
            return new OpenIntake(duration);
        }

        public class StopIntake implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intakeLeft.setPower(0);
                intakeRight.setPower(0);
                return true; // Action complete
            }
        }

        public Action stopIntake() {
            return new StopIntake();
        }
    }


    public class Claw {
        private Servo claw;

        public Claw(HardwareMap hardwareMap) {
            claw = hardwareMap.get(Servo.class, "claw");
        }

        public class CloseClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(0.55); // Change
                return true;
            }
        }

        public Action closeClaw() {
            return new CloseClaw();
        }

        public class OpenClaw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(1.0); // Change
                return true;
            }
        }

        public Action openClaw() {
            return new OpenClaw();
        }
    }




    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        Hortextension hortextension = new Hortextension(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Lift lift = new Lift(hardwareMap);

        waitForStart();

        // Init code
        claw.closeClaw();

        Action DepoSpec;
        Action BasketCycle;
        Action trajectoryBasketCycle;
        Action trajectoryPark;
        Action attachtest;


        /*
        DepoSpec = drive.actionBuilder(drive.pose)
                .lineToX(15)
                .setTangent(Math.toRadians(90))
                .lineToYSplineHeading(53, Math.toRadians(270))
                .turn(Math.toRadians(30))
                .waitSeconds(2)
                .turn(-Math.toRadians(30))
                .build();



         TrajectoryBuilder(beginPose, eps, beginEndVel,
        baseVelConstraint, baseAccelConstraint,
        dispResolution, angResolution,
        pose -> new Pose2dDual<>(
                pose.position.x, pose.position.y.unaryMinus(), pose.heading.inverse()))


        BasketCycle = drive.actionBuilder(drive.pose)

                .lineToY(-17)
                .turn(Math.toRadians(90))

                .build();




        VelConstraint baseVelConstraint = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(50.0),
                new AngularVelConstraint(Math.PI / 2)
        ));
        AccelConstraint baseAccelConstraint = new ProfileAccelConstraint(-10.0, 25.0);





        trajectoryPark = drive.actionBuilder(drive.pose)
                .setTangent(Math.toRadians(30))
                .strafeTo(new Vector2d(14, -10))
                .build();

*/
        attachtest = drive.actionBuilder(drive.pose)
                .setTangent(Math.toRadians(90))
                .lineToY(1)
                .build();


        //Runs code
        Actions.runBlocking(


                new SequentialAction(

                        attachtest,
                        lift.liftUp(),
                        hortextension.extensionforward()


/*
                        hortextension.extensionforward(),
                        intake.openIntake(2000),
                        intake.stopIntake(),
                        hortextension.extensionback(),

 */




/*
                        new ParallelAction(
                                attachtest,
                                new SequentialAction(
                                        lift.liftUp(),
                                        claw.openClaw(),
                                        lift.liftDown()
                                )
                        )

 */



            )
        );
    }


}