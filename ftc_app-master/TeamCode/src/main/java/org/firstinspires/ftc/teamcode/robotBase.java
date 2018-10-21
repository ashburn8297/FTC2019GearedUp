
package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import java.util.ArrayList;

import static android.os.SystemClock.sleep;

/**
Master Robot Config

KNOWN ISSUES
-This config is INCOMPLETE, must be updated for final bot
    -> Namely motor direction, naming, and ease of accessibility
-@TODO imu is unstable, change to I2C system
-@TODO update naming of motors in robotconfig file
Pull this class for anything that has to do with the current robot config
 */
public class robotBase
{
    /* Public OpMode members. */
    public DcMotor leftDrive                = null;
    public DcMotor rightDrive               = null;
    public DcMotor ADM                      = null;
    public DigitalChannel admLim            = null;
    public ModernRoboticsI2cGyro gyro       = null;

    /* local OpMode members. */
    HardwareMap hwMap                       = null;
    private ElapsedTime period              = new ElapsedTime();
    private SamplingOrderDetector detector  = null;
    //Items for encoders
    public static final double  COUNTS_PER_MOTOR_REV = 560.0;
    public static final double  DRIVE_GEAR_REDUCTION = .6;     // This is < 1.0 if geared UP
    public static final double  WHEEL_DIAMETER_INCHES = 4.125;     // For figuring circumference
    public static final double  COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    public static final double  BASE_WIDTH = 16;
    public static final double  DRIVE_SPEED = 0.25;
    public static final double  TURN_SPEED = 0.50;
    public static final double  ENCODER_TURN_COEFF = 1.6;


    public static final int     LEAD_SCREW_TURNS = 20; // Turns in the ADM lead screw

    public static final double  HEADING_THRESHOLD  = 5 ;      // As tight as we can make it with an integer gyro

    /* Constructor */
    
    public robotBase(){

    }
    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        //Setup camera system
        detector = new SamplingOrderDetector();
        detector.init(hwMap.appContext, CameraViewDisplay.getInstance());
        detector.useDefaults();
        detector.downscale = 0.4;

        // Optional Tuning
        detector.areaScoringMethod = DogeCV.AreaScoringMethod.PERFECT_AREA; // Can also be PERFECT_AREA
        detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring

        detector.maxAreaScorer.weight = 0.001;
        detector.ratioScorer.weight = 15;
        detector.ratioScorer.perfectRatio = 1.0;

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

        //Initialize the limit switch
        admLim = hwMap.get(DigitalChannel.class, "ascent_descent_lim");
        gyro = (ModernRoboticsI2cGyro)hwMap.gyroSensor.get("gyro");
    }

    //Drive by encoder
    public void encoderDriveStraight(double inches, double timeoutS, boolean opMode, ElapsedTime runtime) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
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
            while (opMode &&
                    (runtime.seconds() < timeoutS) &&
                    (leftDrive.isBusy() || rightDrive.isBusy())) {
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
    public void encoderDriveTurn(double turnDegrees, double timeoutS, boolean opMode, ElapsedTime runtime) {
        //Possible update, see how many ticks for a full 360 turn
        //Divide this by fraction of a circle to go (degrees to fraction)

        double turningCircle = 3.1415 * BASE_WIDTH;
        double circlePercent= turnDegrees / 360.0;
        double targetDistance = circlePercent * turningCircle * ENCODER_TURN_COEFF;

        int newLeftTarget;
        int newRightTarget;
        // Ensure that the opmode is still active
        if (opMode) {

            // Turn On RUN_TO_POSITION
            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Determine new target position, and pass to motor controller
            newLeftTarget = leftDrive.getCurrentPosition() + (int) (targetDistance * COUNTS_PER_INCH);
            newRightTarget = rightDrive.getCurrentPosition() + (int) (-targetDistance * COUNTS_PER_INCH);
            leftDrive.setTargetPosition(newLeftTarget);
            rightDrive.setTargetPosition(newRightTarget);

            // reset the timeout time and start motion.
            runtime.reset();
            leftDrive.setPower(Math.abs(TURN_SPEED));
            rightDrive.setPower(Math.abs(TURN_SPEED));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opMode &&
                    (runtime.seconds() < timeoutS) &&
                    (leftDrive.isBusy() || rightDrive.isBusy())) {
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
    public void brake() {
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }

    //Drive by gyro
    public void turnByGyro(double angle, double speed, boolean opMode){
        double turnScale = Math.abs((angle-gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle)/angle);
        while (opMode)  {
            float zAngle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            turnScale = Math.abs((angle-gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle)/angle);

            if(zAngle>=(angle-(HEADING_THRESHOLD/2)) && zAngle <= (angle+(HEADING_THRESHOLD/2))){
                rightDrive.setPower(0);
                leftDrive.setPower(0);
                opMode = false;
            }
            else if(zAngle<angle){
                rightDrive.setPower(turnScale*-speed);
                leftDrive.setPower(turnScale*speed);
            }
            else{
                rightDrive.setPower(turnScale*speed);
                leftDrive.setPower(turnScale*-speed);
            }

        }
    }

    //Image Recognition
    public int getMineralPosition(ElapsedTime runtime, double secondOffset){

        ArrayList<Integer> vals = new ArrayList<>();
        int[] orderFreq = new int[4];
        int max=0;
        int maxIndex=0;

        detector.enable();
        while(runtime.seconds()<(1.0+ secondOffset)){
            if(runtime.seconds()>1.0) {
                if (detector.getLastOrder().toString().equals("LEFT")) {
                    vals.add(0);
                } else if (detector.getLastOrder().toString().equals("CENTER")) {
                    vals.add(1);
                } else if (detector.getLastOrder().toString().equals("RIGHT")) {
                    vals.add(2);
                } else {
                    vals.add(-1);
                }
            }

        }

        //After values are added to vals, count each occurrence
        for(int a : vals){
            switch(a) {
                case -1:
                    orderFreq[0]++;
                    break;
                case 0:
                    orderFreq[1]++;
                    break;
                case 1:
                    orderFreq[2]++;
                    break;
                case 2:
                    orderFreq[3]++;
                    break;

            }
        }
        //verify that the right result is returned.
        for(int i=0; i<orderFreq.length; i++){
            if(orderFreq[i]>max){
                maxIndex=i;
                max=orderFreq[i];

            }
        }
        detector.disable();
        return maxIndex;
    }

}