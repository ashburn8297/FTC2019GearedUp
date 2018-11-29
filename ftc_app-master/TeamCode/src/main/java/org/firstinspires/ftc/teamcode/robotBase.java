
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
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
    public DcMotor inVertical               = null; //intake_veritcal
    public DcMotor inHorizontal             = null; //intake_horizontal

    public Servo traverse                   = null; //ADM_servo
    public Servo marker                     = null; //team_marker
    public Servo intakePitch                = null; //intake_pitch
    public DigitalChannel hall              = null; //hall
    public ModernRoboticsI2cGyro gyro       = null; //gyro

    /* local OpMode members. */
    HardwareMap hwMap                       = null;
    private ElapsedTime period              = new ElapsedTime();

    //Items for encoders
    public static final double  COUNTS_PER_MOTOR_REV_neverest = 560.0;
    public static final double  COUNTS_PER_MOTOR_REV_rev      = 560.0;
    public static final double  COUNTS_PER_MOTOR_REV_core     = 288.0;
    public static final double  DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    public static final double  WHEEL_DIAMETER_INCHES = 4.0;    // For figuring circumference

    public static final double  COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV_rev * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    public static final double  DRIVE_SPEED = 0.18;
    public static final double  HEADING_THRESHOLD  = 4;

    public static final int     LEAD_SCREW_TURNS = 19; // Turns in the ADM lead screw

    public static final double maxTraverse = .78;
    public static final double minTraverse = .28;
    public static final double midTraverseRight = .54;
    public static final double midTraverseLeft = .50;

    public static final double markerIn = .19;
    public static final double markerMid = .3;
    public static final double markerOut = .8;

    public static final int armLow = 10;
    public static final int armMid = 50;
    public static final int armHigh = 100;
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
        inHorizontal = hwMap.get(DcMotor.class, "intake_horizontal");
        inVertical = hwMap.get(DcMotor.class, "intake_vertical");

        // Initialize direction of motors
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        ADM.setDirection(DcMotor.Direction.FORWARD);
        inHorizontal.setDirection(DcMotor.Direction.FORWARD);
        inVertical.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        ADM.setPower(0);
        inHorizontal.setPower(0);
        inVertical.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ADM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        inVertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        inVertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        inHorizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        hall = hwMap.get(DigitalChannel.class, "hall");

        traverse = hwMap.get(Servo.class, "ADM_servo");
        traverse.setDirection(Servo.Direction.FORWARD);
        marker = hwMap.get(Servo.class, "team_marker");
        marker.setDirection(Servo.Direction.FORWARD);
        intakePitch = hwMap.get(Servo.class, "intake_pitch");
        intakePitch.setDirection(Servo.Direction.FORWARD);

        gyro = (ModernRoboticsI2cGyro)hwMap.gyroSensor.get("gyro");
    }

    //Auto Methods
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


    public void turnByGyro(double targetAngle, double speed, boolean opMode) {
        float initAngle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        float totalDist = (float)(Math.abs(initAngle)-Math.abs(targetAngle));
        while(opMode){
            float zAngle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

            if ((zAngle >= (targetAngle - (HEADING_THRESHOLD / 2))) && (zAngle <= (targetAngle + (HEADING_THRESHOLD / 2)))) {
                rightDrive.setPower(0);
                leftDrive.setPower(0);
                opMode = false;
            }
            else if(zAngle<targetAngle){
                if(((Math.abs(targetAngle)-Math.abs(zAngle))/totalDist)>.4){
                    speed*=((Math.abs(targetAngle)-Math.abs(zAngle))/totalDist);
                }
                else if(((Math.abs(targetAngle)-Math.abs(zAngle))/totalDist)<.4){
                    speed*=(Math.abs(initAngle)-Math.abs(zAngle))/totalDist;
                }
                rightDrive.setPower(-speed);
                leftDrive.setPower(speed);
            }
            else if(zAngle>targetAngle){
                if(((Math.abs(targetAngle)-Math.abs(zAngle))/totalDist)>.4){
                    speed*=((Math.abs(targetAngle)-Math.abs(zAngle))/totalDist);
                }
                else if(((Math.abs(targetAngle)-Math.abs(zAngle))/totalDist)<.4){
                    speed*=(Math.abs(initAngle)-Math.abs(zAngle))/totalDist;
                }
                rightDrive.setPower(speed);
                leftDrive.setPower(-speed);
            }
        }
    }

    public void brake() {
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }

    public static float getWheelPower(double in){
        int neg = 1;
        in *= 100;
        if(in < 0){
            neg = -1;
        }
        in = Math.abs(in);
        if(in <= 9) in = (in*0.0315)/100;
        else if(in <= 39.537) in = ((0.021 * Math.pow(in-2,2)) + 0.063)/1000;
        else in = ((0.75*in)/100);
        return (float)in * neg;
    }
    public static float getWheelPowerLinear(double in){
        return (float)in;
    }

}

