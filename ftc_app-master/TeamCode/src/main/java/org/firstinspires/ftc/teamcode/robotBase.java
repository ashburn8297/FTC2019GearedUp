
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

import static android.os.SystemClock.sleep;

/**
Master Robot Config

KNOWN ISSUES
-This config is INCOMPLETE, must be updated for final bot
    -> Namely motor direction, naming, and ease of accessibility
Pull this class for anything that has to do with the current robot config
 */
public class robotBase
{
    /* Public OpMode members. */
    public DcMotor leftDrive                = null; //left_drive
    public DcMotor rightDrive               = null; //right_drive
    public DcMotor ADM                      = null; //ascent_descent

    public Servo traverse                   = null; //ADM_servo
    public DigitalChannel admLim            = null; //ascent_descent_lim
    public ModernRoboticsI2cGyro gyro       = null; //gyro

    /* local OpMode members. */
    HardwareMap hwMap                       = null;
    private ElapsedTime period              = new ElapsedTime();

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final String VUFORIA_KEY = "AbEDH9P/////AAABmcFPgUDLz0tMh55QD8t9w6Bqxt3h/G+JEMdItgpjoR+S1FFRIeF/w2z5K7r/nUzRZKleksLHPglkfMKX0NltxxpVUpXqj+w6sGvedaNq449JZbEQxaYe4SU+3NNi0LBN879h9LZW9RxJFOMt7HfgssnBdg+3IsiwVKKYnovU+99oz3gJkcOtYhUS9ku3s0Wz2n6pOu3znT3bICiR0/480N63FS7d6Mk6sqN7mNyxVcRf8D5mqIMKVNGAjni9nSYensl8GAJWS1vYfZ5aQhXKs9BPM6mST5qf58Tg4xWoHltcyPp0x33tgQHBbcel0M9pYe/7ub1pmzvxeBqVgcztmzC7uHnosDO3/2MAMah8qijd";
    private TFObjectDetector tfod;
    private VuforiaLocalizer vuforia;

    //Items for encoders
    public static final double  COUNTS_PER_MOTOR_REV = 560.0;
    public static final double  DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    public static final double  WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    public static final double  COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    public static final double  DRIVE_SPEED = 0.25;

    public static final int     LEAD_SCREW_TURNS = 17; // Turns in the ADM lead screw

    public static final double  HEADING_THRESHOLD  = 5 ;

    public static final double maxTraverse = .79;
    public static final double minTraverse = .28;
    public static final double midTraverse = .54;
    /* Constructor */

    public robotBase(){

    }
    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftDrive = hwMap.get(DcMotor.class, "left_drive");
        rightDrive = hwMap.get(DcMotor.class, "right_drive");
        ADM = hwMap.get(DcMotor.class, "ascent_descent"); //Control Ascent Descent Module (ADM)

        // Initialize direction of motors
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        ADM.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        ADM.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ADM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        admLim = hwMap.get(DigitalChannel.class, "ascent_descent_lim");

        traverse = hwMap.get(Servo.class, "ADM_servo"); //CRServo is continuous rotation servo
        traverse.setDirection(Servo.Direction.FORWARD);

        gyro = (ModernRoboticsI2cGyro)hwMap.gyroSensor.get("gyro");
    }
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;


        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }
    private void initTfod() {
        int tfodMonitorViewId = hwMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hwMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    //Auto Methods
    public void encoderDriveStraight(double inches, double timeoutS, boolean opMode, ElapsedTime runtime) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        inches = -inches;
        if (opMode) {

            // Turn On RUN_TO_POSITION
            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Determine new target position, and pass to motor controller
            newLeftTarget = leftDrive.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
            newRightTarget = rightDrive.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);
            leftDrive.setTargetPosition(newLeftTarget);
            rightDrive.setTargetPosition(newRightTarget);

            // reset the timeout time and start motion.
            runtime.reset();
            leftDrive.setPower(Math.abs(DRIVE_SPEED));
            rightDrive.setPower(Math.abs(DRIVE_SPEED));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opMode && (runtime.seconds() < timeoutS) && (leftDrive.isBusy() || rightDrive.isBusy())) {

            }

            // Stop all motion;
            brake();

            leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Turn off RUN_TO_POSITION
            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move
        }

    }
    public void turnByGyro(double angle, double speed, boolean opMode) {
        double turnScale;

        while (opMode) {
            float zAngle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            turnScale = Math.abs((angle - gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle) / angle);

            if ((zAngle >= (angle - (HEADING_THRESHOLD / 2))) && (zAngle <= (angle + (HEADING_THRESHOLD / 2)))) {
                rightDrive.setPower(0);
                leftDrive.setPower(0);
                opMode = false;
            } else if (zAngle < angle) {
                rightDrive.setPower(turnScale * -speed);
                leftDrive.setPower(turnScale * speed);
            } else {
                rightDrive.setPower(turnScale * speed);
                leftDrive.setPower(turnScale * -speed);
            }

        }
    }
    public int track(ElapsedTime runtime){
        initVuforia();
        int[] orderFreq = new int[3];
        int maxIndex = 0;
        int max = 0;
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        }
        if (tfod != null) {
            tfod.activate();
        }

        while ((runtime.seconds() < 4) && (runtime.seconds() > 1)) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    if (updatedRecognitions.size() == 3) {
                        int goldMineralX = -1;
                        int silverMineral1X = -1;
                        int silverMineral2X = -1;
                        for (Recognition recognition : updatedRecognitions) {
                            if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                goldMineralX = (int) recognition.getLeft();
                            } else if (silverMineral1X == -1) {
                                silverMineral1X = (int) recognition.getLeft();
                            } else {
                                silverMineral2X = (int) recognition.getLeft();
                            }
                        }

                        //0 is left, 2 is right, 1 is center
                        if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                            if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                                orderFreq[0]++;
                            } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                orderFreq[2]++;
                            } else {
                                orderFreq[1]++;
                            }
                        }
                    }
                }
            }
        }
        if (tfod != null) {
            tfod.shutdown();
        }
        for(int i=0; i<orderFreq.length; i++) {
            if (orderFreq[i] > max) {
                maxIndex = i;
                max = orderFreq[i];
            }
        }
        return maxIndex;
    }
    public void brake() {
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }

    public static float getWheelPower(double in){
        if(in > .02) in = 1 / (1 + Math.pow(2.7182, (-4 * ((2 * in) - 1))));
        else if(in < -.02) in = -(1 / (1 + Math.pow(2.7182, (-4 * ((-2 * in) - 1)))));
        else in = 0.0;

        return (float)in;
    }
    public static float getWheelPowerLinear(double in){
        return (float)in;
    }

}

