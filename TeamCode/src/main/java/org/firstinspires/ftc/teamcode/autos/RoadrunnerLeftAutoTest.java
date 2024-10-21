package org.firstinspires.ftc.teamcode.autos;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.tuning.MecanumDrive;

@Config
@Autonomous(name = "RoadrunnerLeftAutoTest", group = "Autonomous")
public class RoadrunnerLeftAutoTest extends LinearOpMode {

    // Vertical Extension
    public class Vertextension {
        private DcMotor vertextension;

        public Vertextension(HardwareMap hardwareMap) {
            vertextension = hardwareMap.get(DcMotor.class, "extendcentral");
            vertextension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            vertextension.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        public Action extensionforward() {
            return new Extensionforward();
        }

        public class Extensionforward implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    vertextension.setPower(0.8);
                    initialized = true;
                }

                NormalizedColorSensor blocksensor= hardwareMap.get(NormalizedColorSensor.class, "blocksensor");
                DistanceSensor distanceblocksensor = (DistanceSensor) blocksensor;

                double pos = vertextension.getCurrentPosition();
                packet.put("vertical extension forward", pos);

                if (pos >= 2700 && pos <= 3000 || distanceblocksensor.getDistance(DistanceUnit.CM) < 3) {
                    vertextension.setPower(0);
                    return true;
                } else {
                    vertextension.setPower(0.8);
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
                    vertextension.setPower(-0.45); // Change accordingly
                    initialized = true;
                }

                TouchSensor slidesensor;
                slidesensor = hardwareMap.get(TouchSensor.class, "slidesensor");
                double pos = vertextension.getCurrentPosition();
                packet.put("vertextension back", pos);

                if (pos < 100.0 || slidesensor.isPressed()) {
                    vertextension.setPower(0);
                    return true;
                } else {
                    vertextension.setPower(-0.45); // Change accordingly
                    return false;
                }
            }
        }
    }

    // Extend Intake
    public class Extendintake {
        private DcMotor extendintake;

        public Extendintake(HardwareMap hardwareMap) {
            extendintake = hardwareMap.get(DcMotor.class, "extendintake");
            extendintake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            extendintake.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        public Action extendintake() {
            return new ExtensionIntake();
        }

        public class ExtensionIntake implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    extendintake.setPower(0.8);
                    initialized = true;
                }
                NormalizedColorSensor intakesensor = hardwareMap.get(NormalizedColorSensor.class, "blocksensor");
                DistanceSensor distanceintakesensor = (DistanceSensor) intakesensor;

                long ElapsedTime = System.currentTimeMillis();

                while ((ElapsedTime + 1500) > (System.currentTimeMillis())) {
                    if (!(distanceintakesensor.getDistance(DistanceUnit.CM) < 3)) {
                        extendintake.setPower(0.8);
                        return true;
                    } else {
                        extendintake.setPower(0.8);
                        return false;
                    }
                }
                extendintake.setPower(0);
                return true;
            }
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
        Vertextension vertextension = new Vertextension(hardwareMap);
        Extendintake extendintake = new Extendintake(hardwareMap);
        Claw claw = new Claw(hardwareMap);

        // Init code
        claw.openClaw();
        vertextension.extensionback();

        Action trajectoryDepo;
        Action trajectoryCollection;
        Action trajectoryBasketCycle;
        Action trajectoryPark;

        trajectoryDepo = drive.actionBuilder(drive.pose)
                .lineToX(15)
                .setTangent(Math.toRadians(90))
                .lineToYSplineHeading(53, Math.toRadians(270))
                .turn(Math.toRadians(30))
                .waitSeconds(2)
                .turn(-Math.toRadians(30))
                .build();

        trajectoryBasketCycle = drive.actionBuilder(drive.pose)
                .lineToY(-17)
                .turn(Math.toRadians(40))
                .waitSeconds(2)
                .turn(-Math.toRadians(40))
                .lineToYSplineHeading(53, Math.toRadians(270))
                .turn(Math.toRadians(30))
                .waitSeconds(2)
                .turn(-Math.toRadians(30))
                .build();

        trajectoryPark = drive.actionBuilder(drive.pose)
                .setTangent(Math.toRadians(30))
                .strafeTo(new Vector2d(14, -10))
                .build();

        Actions.runBlocking(
                new SequentialAction(
                        vertextension.extensionforward(),
                        extendintake.extendintake(),
                        vertextension.extensionback(),
                        claw.closeClaw()
                )
        );
    }
}
