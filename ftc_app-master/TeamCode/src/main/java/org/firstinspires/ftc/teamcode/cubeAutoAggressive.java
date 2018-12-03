package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import net.frogbots.ftcopmodetunercommon.opmode.TunableLinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import static org.firstinspires.ftc.teamcode.robotBase.midTraverseRight;


@Autonomous(name = "CubeAuto AGGRESSIVE")
@Disabled
public class cubeAutoAggressive extends TunableLinearOpMode {
    robotBase robot = new robotBase();
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
        //tensor flow IR start
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }


        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();

        waitForStart();
        runtime.reset();
        telemetry.update();

        robot.ADM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.ADM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.inVertical.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Raise ADM, and drop from lander

        robot.ADM.setTargetPosition((int)(robot.LEAD_SCREW_TURNS * robot.COUNTS_PER_MOTOR_REV_rev)-100); //tuner
        robot.ADM.setPower(.5);
        telemetry.addData("Lift Encoder Value", robot.ADM.getCurrentPosition());

        sleep(5000);

        //Move lift left
        robot.traverse.setPosition(robot.maxTraverse);
        sleep(1000);

        robot.marker.setPosition(robot.markerMid);
        sleep(500);
        robot.intakePitch.setPosition(robot.boxFlat);
        robot.intake.setPower(-1);
        sleep(500);

        /*//////////////////////////////////////////


        ARM MOTOR IS REVERSED SO NEGATIVE IS UP!!!!!!!!!!!!!!!!!!!!!!!!!!!!

        /////////////////////////////////////////*/
        robot.inVertical.setPower(-.45);
        sleep(2000);
        robot.inVertical.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.inVertical.setPower(0);

        //Vuforia command
        if (tfod != null) {
            tfod.activate();
        }
        runtime.reset();
        while (runtime.seconds() < 4) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    if (updatedRecognitions.size() == 2) {
                        int goldMineralX = -1;
                        int silverMineral1X = -1;
                        for (Recognition recognition : updatedRecognitions) {
                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                goldMineralX = (int) recognition.getLeft();
                            } else {
                                silverMineral1X = (int) recognition.getLeft();
                            }
                        }
                        if(goldMineralX == -1){
                            freq[0]++;
                        }
                        else if (silverMineral1X != -1) {
                            if (goldMineralX < silverMineral1X) {
                                freq[1]++;
                            } else {
                                freq[2]++;
                            }
                        }
                    }
                }
            }
        }
        for(int i=0; i<freq.length; i++) {
            if (freq[i] > max) {
                maxIndex = i;
                max = freq[i];
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }

        //Lower arm
        robot.inVertical.setPower(.40);
        sleep(2000);
        robot.inVertical.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.inVertical.setPower(0);

        robot.marker.setPosition(robot.markerIn);
        sleep(500);

        telemetry.addData("Location", maxIndex);
        telemetry.update();

        sleep(500);

        robot.encoderDriveStraight(10, 2.0, opModeIsActive(), runtime);

        if(maxIndex == 0) {
            robot.turnByGyro(45, .2, opModeIsActive());
            robot.encoderDriveStraight(32, 2.0, opModeIsActive(), runtime);
            robot.turnByGyro(-45, .2, opModeIsActive());
            robot.encoderDriveStraight(36, 2.0, opModeIsActive(), runtime);
            robot.intake.setPower(0.0);
        }
        if(maxIndex == 1) {
            robot.encoderDriveStraight(50, 4.0, opModeIsActive(), runtime);
        }
        if(maxIndex == 2) {
            robot.turnByGyro(-45, .2, opModeIsActive());
            robot.encoderDriveStraight(32, 2.0, opModeIsActive(), runtime);
            robot.turnByGyro(45, .2, opModeIsActive());
            robot.encoderDriveStraight(36, 2.0, opModeIsActive(), runtime);
            robot.intake.setPower(0.0);
        }
        robot.turnByGyro(45, .2, opModeIsActive());
        robot.marker.setPosition(robot.markerOut);
        robot.encoderDriveStraight(-80, 8, opModeIsActive(), runtime);

        stop();
    }
    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
}