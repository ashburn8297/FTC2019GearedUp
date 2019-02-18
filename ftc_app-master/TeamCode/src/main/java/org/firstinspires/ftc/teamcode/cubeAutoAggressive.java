package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

import static org.firstinspires.ftc.teamcode.robotBase.midTraverseRight;


@Autonomous(name = "CubeAuto AGGRESSIVE")
//@Disabled
public class cubeAutoAggressive extends LinearOpMode {
    robotBaseAuto robot = new robotBaseAuto();
    private ElapsedTime runtime = new ElapsedTime();

    int maxIndex = 0;
    int[] freq = new int[3];
    int max = 0;

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private static final String VUFORIA_KEY = "AbEDH9P/////AAABmcFPgUDLz0tMh55QD8t9w6Bqxt3h/G+JEMdItgpjoR+S1FFRIeF/w2z5K7r/nUzRZKleksLHPglkfMKX0NltxxpVUpXqj+w6sGvedaNq449JZbEQxaYe4SU+3NNi0LBN879h9LZW9RxJFOMt7HfgssnBdg+3IsiwVKKYnovU+99oz3gJkcOtYhUS9ku3s0Wz2n6pOu3znT3bICiR0/480N63FS7d6Mk6sqN7mNyxVcRf8D5mqIMKVNGAjni9nSYensl8GAJWS1vYfZ5aQhXKs9BPM6mST5qf58Tg4xWoHltcyPp0x33tgQHBbcel0M9pYe/7ub1pmzvxeBqVgcztmzC7uHnosDO3/2MAMah8qijd";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        robot.init(hardwareMap);
        robot.traverse.setPosition(midTraverseRight);
        robot.ADM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.ADM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //tensor flow IR start
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        telemetry.update();

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();



        waitForStart();

        //Lower Lift
        if (opModeIsActive()) {
            robot.ADM.setTargetPosition((int) (robot.LEAD_SCREW_TURNS * robot.COUNTS_PER_MOTOR_REV_rev) - 100); //tuner
            robot.ADM.setPower(1.0);
        }

        sleep(3000);
        robot.ADM.setPower(.05); //To stop jittering

        if(opModeIsActive()){
            robot.traverse.setPosition(robot.maxTraverse);
        }

        runtime.reset();

        if (opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }

            while (opModeIsActive()&&runtime.seconds()<5) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        if (updatedRecognitions.size() >= 2) {
                            int goldMineralX = -1;
                            int goldMineralY = -1;
                            int silverMineral1X = -1;
                            int silverMineral2X = -1;
                            for (Recognition recognition : updatedRecognitions) {
                                if(recognition.getTop() > 500) {
                                    if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                        goldMineralX = (int) recognition.getLeft();
                                        goldMineralY = (int) recognition.getTop();
                                    } else if (silverMineral1X == -1) {
                                        silverMineral1X = (int) recognition.getLeft();
                                    } else {
                                        silverMineral2X = (int) recognition.getLeft();
                                    }
                                }
                            }
                            if ((silverMineral1X != -1 && silverMineral2X != -1) || goldMineralX == -1) {
                                freq[2]++;
                            }
                            else if (goldMineralX < silverMineral1X || goldMineralX < silverMineral2X) {
                                freq[0]++;
                            }
                            else if (goldMineralX > silverMineral1X || goldMineralX > silverMineral2X){
                                freq[1]++;
                            }
                            telemetry.addData("Gold", goldMineralX);
                            telemetry.addData("Gold Y", goldMineralY);
                            telemetry.addData("S1", silverMineral1X);
                            telemetry.addData("S2", silverMineral2X);
                        }
                        telemetry.update();
                    }
                }
            }
        }
        for (int i = 0; i < freq.length; i++) {
            if (freq[i] > max) {
                maxIndex = i;
                max = freq[i];
            }
        }
        if (tfod != null) {
            tfod.shutdown();
        }
        telemetry.addData("Location", maxIndex);
        telemetry.addData("Index 0", freq[0]);
        telemetry.addData("Index 1", freq[1]);
        telemetry.addData("Index 2", freq[2]);
        telemetry.update();


        //Slide over
        if (opModeIsActive()) {
            robot.encoderDriveStraight(4, 1.0, opModeIsActive(), runtime);
        }

        if(maxIndex == 0) {
            robot.turnByEncoder(30, .09, opModeIsActive(), 3.0, runtime);
            robot.encoderDriveStraight(36, 3.0, opModeIsActive(), runtime);
            robot.encoderDriveCrater(-2, 1.0, opModeIsActive(), runtime);
            robot.turnByEncoder(-80, .09, opModeIsActive(), 3.0, runtime);
            robot.encoderDriveStraight(30, 2.0, opModeIsActive(), runtime);
            robot.turnByEncoder(90, .11, opModeIsActive(), 3.0, runtime);
        }
        else if(maxIndex == 1){
            robot.turnByEncoder(0, .09, opModeIsActive(), 3.0, runtime);
            robot.encoderDriveStraight(51, 3.0, opModeIsActive(), runtime);
            robot.turnByEncoder(45, .09, opModeIsActive(), 3.0, runtime);

        }
        else if(maxIndex == 2) {
            robot.turnByEncoder(-32, .09, opModeIsActive(), 3.0, runtime);
            robot.encoderDriveStraight(40, 3.0, opModeIsActive(), runtime);
            robot.encoderDriveCrater(-3, 1.5, opModeIsActive(), runtime);
            robot.turnByEncoder(75, .09, opModeIsActive(), 3.0, runtime);
            robot.encoderDriveStraight(30, 2.0, opModeIsActive(), runtime);
        }


        //Drop off marker (out, in)
        if (opModeIsActive()) {
            robot.marker.setPosition(robot.markerOut);
            sleep(1000);
            robot.marker.setPosition(robot.markerMid);
        }

        robot.turnByEncoder(-100, .09, opModeIsActive(), 3.0, runtime);
        robot.encoderDriveStraight(-10, 1.0, opModeIsActive(), runtime);
        sleep(1000);
    }
    private void initVuforia () {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.useExtendedTracking = false;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        //or
        //parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }
    private void initTfod () {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = .6;

        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
}