package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.List;
@TeleOp(name = "LimeTest", group = "Sensor")
public class LimelightTesting extends OpMode {

    private Limelight3A limelight;
    private DcMotor lm, rm,blm,brm;
    double closestObjectX;
    boolean foundObject = false;
    LLResult result;



    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!
        limelight.pipelineSwitch(0); // Switch to pipeline number 0
        //  result.getPipelineIndex()
        lm = hardwareMap.get(DcMotorEx.class, "lm");
        blm = hardwareMap.get(DcMotorEx.class, "blm");
        brm = hardwareMap.get(DcMotorEx.class, "brm");
        rm = hardwareMap.get(DcMotorEx.class, "rm");
      double closestObjectX = limeLightColorX(limeLightResult());
    }

    @Override
    public void loop() {
       /* if(foundObject != true){
             closestObjectX = limeLightColorX(limeLightResult());
        }

        if(closestObjectX < 0 ){
            /*robot. strafeLeft(0.2);

        }
        else if(closestObjectX > 0){
            strafeRight(0.2);
        }

        */
        result = limelight.getLatestResult();
        if (result != null) {

            if (result.isValid()) {
                telemetry.addData("tx", result.getTx());
                telemetry.addData("txnc", result.getTxNC());
                telemetry.addData("ty", result.getTy());
                telemetry.addData("tync", result.getTyNC());
            }
            if(result.getTx() < 0){
                telemetry.addData("Direction:", "right");
            }
            else if (result.getTx() > 0){
                telemetry.addData("Direction:", "right");

            }
            telemetry.update();
        }




        // print some data for each detected target

    }

    private LLResult limeLightResult( ) {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            double tx = result.getTx(); // How far left or right the target is (degrees)
            double ty = result.getTy(); // How far up or down the target is (degrees)
            double ta = result.getTa(); // How big the target looks (0%-100% of the image)

            telemetry.addData("Target X", tx);
            telemetry.addData("Target Y", ty);
            telemetry.addData("Target Area", ta);
        } else {
            telemetry.addData("Limelight", "No Targets");
        }
        telemetry.update();

        return result;

    }
    private void strafeLeft(double power){
        rm.setPower(-power);
        brm.setPower(power);
        lm.setPower(power);
        blm.setPower(-power);

    }

    private void strafeRight(double power){
        rm.setPower(power);
        brm.setPower(-power);
        lm.setPower(-power);
        blm.setPower(power);

    }


   /* private boolean isBlue(LLResultTypes.ColorResult cr){
        if(cr.)
        return false;
    }
}*/

    private double limeLightColorX(LLResult result) {
        int closestColorX = 1000;
        int closestColorY = 1000;
        if (result.isValid()) {
            foundObject = true;
            // Access color results
            List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
            for (LLResultTypes.ColorResult cr : colorResults) {
                int tarX =  (int)cr.getTargetXPixels();
                int objectIndex = colorResults.indexOf(cr);
                if(closestColorX > tarX){
                    closestColorX = tarX;
                    telemetry.addData("Color", "X: %.2f, Y: %.2f", colorResults.get(objectIndex).getTargetXPixels());
                    telemetry.update();
                }
                return colorResults.get(objectIndex).getTargetXPixels();



            }

        }

        return 0;
    }



}
