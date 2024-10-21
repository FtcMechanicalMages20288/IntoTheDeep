package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;



/*
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="mecanumDrive", group="Linear OpMode")

public class mecanumDrive extends LinearOpMode {

    robotManager robot;


    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        robot = new robotManager();
        robot.InitializeRobot(hardwareMap, telemetry);

        while (opModeIsActive()) {
            /*
            GAMEPAD 1 CONTROLS (DRIVER)

            L-Stick - Move
            R-Stick - Rotate
            RB - Intake
            LB - Outtake
            X - First bar hang
            Y - Second bar hang
             */
            //moves robot based on the input (field centric)
            if (Math.abs(gamepad1.left_stick_y) < 0.2 && Math.abs(gamepad1.left_stick_x) < 0.2 && Math.abs(gamepad1.right_stick_x) < 0.2) {
                robot.SetMotors(0, 0, 0, 0);
            }
            else {
                robot.MoveRobot(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, 1);
            }

            //intake/outtake
            if (gamepad1.right_bumper) {
                robot.SetIntake(1.0);
            }
            else if (gamepad1.left_bumper) {
                robot.SetIntake(-1.0);
            }
            else {
                robot.SetIntake(0.0);
            }

            /*
            GAMEPAD 2 CONTROLS (ATTACHMENTS)

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

            //move claw
            if (gamepad2.y) {
                robot.MoveClawToPosition(1);
            }
            else if (gamepad2.b) {
                robot.MoveClawToPosition(2);
            }
            else if (gamepad2.a) {
                robot.MoveClawToPosition(4);
            }
            else if (gamepad2.x) {
                robot.MoveClawToPosition(3);
            }

            //open/close claw
            if (gamepad2.right_bumper) {
                robot.CloseClaw();
            }
            else if (gamepad2.left_bumper) {
                robot.OpenClaw();
            }

            //extend/retract horizontal
            if (gamepad2.right_trigger > 0.5) {
                robot.ExtendHorizontal();
            }
            else if (gamepad2.left_trigger > 0.5) {
                robot.RetractHorizontal();
            }

            //lower intake
            if (gamepad2.dpad_down) {
                robot.IntakeDown();
            }
        }
    }


}
