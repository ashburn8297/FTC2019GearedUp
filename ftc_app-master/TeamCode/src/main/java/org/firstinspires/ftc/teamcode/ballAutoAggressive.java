package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import static org.firstinspires.ftc.teamcode.robotBase.midTraverseRight;


@Autonomous(name = "BallAuto AGGRESSIVE")
//@Disabled
public class ballAutoAggressive extends LinearOpMode {
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
        robot.inVertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.inVertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Raise ADM, and drop from lander

        robot.ADM.setTargetPosition((int)(robot.LEAD_SCREW_TURNS * robot.COUNTS_PER_MOTOR_REV_rev)-100); //tuner
        robot.ADM.setPower(.5);
        telemetry.addData("Lift Encoder Value", robot.ADM.getCurrentPosition());
        //robot.admMove((int)(robot.LEAD_SCREW_TURNS * robot.COUNTS_PER_MOTOR_REV_rev)-100);

        sleep(4000);
        //Move ADM to the side, move marker, keep box raised
        robot.traverse.setPosition(robot.maxTraverse);
        sleep(500);
        robot.marker.setPosition(robot.markerMid);
        sleep(500);
        robot.intakePitch.setPosition(robot.boxFlat);
        robot.intake.setPower(-1);
        sleep(500);
        robot.marker.setPosition(robot.markerIn);
        sleep(500);
        robot.ADM.setTargetPosition(50);
        robot.ADM.setPower(-.5);
        sleep(4000);
        //if(opModeIsActive()) {
          //  robot.ADM.setTargetPosition(50);
          //  robot.ADM.setPower(-.5);
        //}
        sleep(1000);
        robot.encoderDriveStraight(18, 3, opModeIsActive(), runtime);
        robot.intake.setPower(0);

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
