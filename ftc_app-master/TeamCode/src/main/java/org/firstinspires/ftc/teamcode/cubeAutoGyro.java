package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

import static org.firstinspires.ftc.teamcode.robotBase.midTraverseRight;

@Autonomous(name = "CubeAuto GYRO")

public class cubeAutoGyro extends LinearOpMode {

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

        while (robot.navxMicro.isCalibrating())  {
            telemetry.addData("calibrating", "%s", Math.round(runtime.seconds())%2==0 ? "|.." : "..|");
            telemetry.update();
            sleep(50);
        }
        telemetry.log().clear(); telemetry.log().add("Gyro Calibrated. Press Start.");
        robot.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        telemetry.clear(); telemetry.update();

        waitForStart();

        robot.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE);
        //Lower Lift
        if (opModeIsActive()) {
            robot.ADM.setTargetPosition((int) (robot.LEAD_SCREW_TURNS * robot.COUNTS_PER_MOTOR_REV_rev) - 100); //tuner
            robot.ADM.setPower(.95);
        }

        sleep(3000);
        robot.ADM.setPower(.05); //To stop jittering

        if(opModeIsActive()){
            robot.traverse.setPosition(robot.maxTraverse);
        }

        runtime.reset();
        robot.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_WHITE);
        if (opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }

            while (opModeIsActive()&&runtime.seconds()<1.5) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        if (updatedRecognitions.size() >= 2) {
                            int goldMineralX = -1;
                            int silverMineral1X = -1;
                            int silverMineral2X = -1;
                            for (Recognition recognition : updatedRecognitions) {
                                if(recognition.getTop() > 280) {
                                    if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                        goldMineralX = (int) recognition.getLeft();
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
                            telemetry.addData("S1", silverMineral1X);
                            telemetry.addData("S2", silverMineral2X);
                            robot.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_BLUE);
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
        telemetry.addData("Left", freq[0]);
        telemetry.addData("Center", freq[1]);
        telemetry.addData("Right", freq[2]);
        telemetry.update();
        
        /**
         * Speed #'s need to be fixed up. Before all were .24, now new numbers take effect
         */
        robot.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.GOLD);
        if (opModeIsActive()) {
            robot.encoderDriveStraight(4, 1.0, .20,opModeIsActive(), runtime);
        }

        if(maxIndex == 0) {
            robot.turnByGyro(35, .18, opModeIsActive(), 1.25, runtime);
            robot.encoderDriveStraight(25, 1.5, .25, opModeIsActive(), runtime);
        }
        else if(maxIndex == 1){
            robot.encoderDriveStraight(20, 1.0, .25, opModeIsActive(), runtime);
        }
        else if(maxIndex == 2){
            robot.turnByGyro(-35, .18, opModeIsActive(), 1.25, runtime);
            robot.encoderDriveStraight(25, 1.5, .25, opModeIsActive(), runtime);
        }

        robot.encoderDriveStraight(-10, 1.5, .2, opModeIsActive(), runtime);
        robot.turnByGyro(90, .15, opModeIsActive(), 2.0, runtime);

        if(maxIndex ==0) {
            robot.encoderDriveStraight(25, 2.5, .23, opModeIsActive(), runtime);
        }
        if(maxIndex ==1 ) {
            robot.encoderDriveStraight(30, 2.5, .23, opModeIsActive(), runtime);
        }
        if(maxIndex ==2) {
            robot.encoderDriveStraight(38, 2.5, .26, opModeIsActive(), runtime);
        }

        robot.turnByGyro(-135, .23, opModeIsActive(), 3.0, runtime);
        robot.encoderDriveStraight(-22, 2.0, .25, opModeIsActive(), runtime);
        robot.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.FIRE_LARGE);
        robot.encoderDriveStraight(4, .5, .15, opModeIsActive(), runtime);
        robot.turnByGyro(-45, .15, opModeIsActive(), 2.0, runtime);
        robot.encoderDriveRamp(53, 3.0, .45, opModeIsActive(), runtime);

        robot.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
        if (opModeIsActive()) {
            robot.marker.setPosition(robot.markerOut);
            sleep(700);
            robot.marker.setPosition(robot.markerMid);
        }
        robot.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.FIRE_LARGE);
        robot.turnByGyro(-55, .40, opModeIsActive(), 1.0, runtime);
        robot.encoderDriveRamp(-65, 3.5, .45, opModeIsActive(), runtime);
        //some sort of loop to check and see if robot has collided with crater wall
        runtime.reset();
        robot.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_WHITE);
        while(Math.abs(robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).thirdAngle)<3 && (runtime.seconds()<2) && opModeIsActive()){
            robot.leftDrive.setPower(-.1);
            robot.rightDrive.setPower(-.1);
        }
        robot.brake();
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
