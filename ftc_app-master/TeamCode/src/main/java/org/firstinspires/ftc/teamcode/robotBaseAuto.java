
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import static android.os.SystemClock.sleep;

/**
Master Robot Config

KNOWN ISSUES
-This config is INCOMPLETE, must be updated for final bot
    -> Namely motor direction, naming, and ease of accessibility
Pull this class for anything that has to do with the current robot config
 */

public class robotBaseAuto
{
    /* Public OpMode members. */
    public DcMotor leftDrive                = null; //left_drive
    public DcMotor rightDrive               = null; //right_drive
    public DcMotor ADM                      = null; //ascent_descent
    public DcMotor inVertical               = null; //intake_veritcal
    public DcMotor inHorizontal             = null; //intake_horizontal

    public Servo traverse                   = null; //ADM_servo
    public Servo marker                     = null; //team_marker
    public Servo intakeGate                 = null; //intake_pitch
    public CRServo intake                   = null; //intake
    public DigitalChannel hall              = null; //
    public DigitalChannel vertHall          = null; //vertHall

    IntegratingGyroscope gyro;

    NavxMicroNavigationSensor navxMicro;

    /* local OpMode members. */
    HardwareMap hwMap                       = null;
    private ElapsedTime period              = new ElapsedTime();

    //Items for encoders
    public static final double  COUNTS_PER_MOTOR_REV_rev      = 560.0;
    public static final double  DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    public static final double  WHEEL_DIAMETER_INCHES = 4.0;    // For figuring circumference

    public static final double  COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV_rev * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    public static final double  DRIVE_SPEED = 0.24; //.23
    public static final double  DRIVE_SPEED_CRATER = 0.1; //.23
    public static final double  HEADING_THRESHOLD  = 2;

    public static final double  LEAD_SCREW_TURNS = 12.25; // Turns in the ADM lead screw

    public static final double maxTraverse = .73;
    public static final double minTraverse = .32;
    public static final double midTraverseRight = .56;
    public static final double midTraverseLeft = .48;

    public static final double markerIn = .15;
    public static final double markerMid = .3;
    public static final double markerOut = .8;

    public static final double ticksPerDegree = 1800/360;

    /* Constructor */

    public robotBaseAuto(){

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
        inHorizontal.setDirection(DcMotor.Direction.REVERSE);
        inVertical.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        ADM.setPower(0);
        inHorizontal.setPower(0);
        inVertical.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ADM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //inVertical.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        inVertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        inHorizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        hall = hwMap.get(DigitalChannel.class, "hall");
        vertHall = hwMap.get(DigitalChannel.class, "vertHall");

        traverse = hwMap.get(Servo.class, "ADM_servo");
        traverse.setDirection(Servo.Direction.FORWARD);
        intake = hwMap.get(CRServo.class, "intake");
        intake.setDirection(CRServo.Direction.FORWARD);
        marker = hwMap.get(Servo.class, "team_marker");
        marker.setDirection(Servo.Direction.FORWARD);
        intakeGate = hwMap.get(Servo.class, "intake_pitch");
        intakeGate.setDirection(Servo.Direction.FORWARD);



        navxMicro = hwMap.get(NavxMicroNavigationSensor.class, "navx");
        gyro = navxMicro;
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

            sleep(500);   // optional pause after each move
        }

    }
    public void encoderDriveCrater(double inches, double timeoutS, boolean opMode, ElapsedTime runtime) {
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
            leftDrive.setPower(Math.abs(DRIVE_SPEED_CRATER));
            rightDrive.setPower(Math.abs(DRIVE_SPEED_CRATER));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opMode && (runtime.seconds() < timeoutS) && (leftDrive.isBusy() || rightDrive.isBusy())) {

            }
            // Turn off RUN_TO_POSITION
            brake();

            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(500);   // optional pause after each move
        }
    }


        public void turnByGyro(double targetAngle, double speed, boolean opMode, double timeoutS, ElapsedTime runtime) {
        runtime.reset();
        while(opMode && runtime.seconds() <timeoutS){
            float CurAngle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

            if ((CurAngle >= (targetAngle - (HEADING_THRESHOLD / 2))) && (CurAngle <= (targetAngle + (HEADING_THRESHOLD / 2)))) {
                rightDrive.setPower(0);
                leftDrive.setPower(0);
                opMode = false;
            }
            else if(CurAngle<targetAngle){
                rightDrive.setPower(-speed);
                leftDrive.setPower(speed);
            }
            else if(CurAngle>targetAngle){
                rightDrive.setPower(speed);
                leftDrive.setPower(-speed);
            }
        }
        brake();
    }

    public void turnByEncoder(double targetAngle, double speed, boolean opMode, double timeoutS, ElapsedTime runtime){
        int newLeftTarget;
        int newRightTarget;
        int targetTicks = (int)(targetAngle*ticksPerDegree);

        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Determine new target position, and pass to motor controller
        newLeftTarget = leftDrive.getCurrentPosition() + targetTicks;
        newRightTarget = rightDrive.getCurrentPosition() - targetTicks;
        leftDrive.setTargetPosition(newLeftTarget);
        rightDrive.setTargetPosition(newRightTarget);

        // reset the timeout time and start motion.
        runtime.reset();
        leftDrive.setPower(Math.abs(speed));
        rightDrive.setPower(Math.abs(speed));
        while (opMode && (runtime.seconds() < timeoutS) && (leftDrive.isBusy() || rightDrive.isBusy())) {

        }

        // Stop all motion;
        brake();

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Turn off RUN_TO_POSITION
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        brake();
    }

    public void brake() {
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }

    public static double getWheelPower(double in){
        in *= .27;
        return in;
    }

    public static float getWheelPowerLinear(double in){
        return (float)in;
    }

    String formatDegrees(double degrees){
        return String.format("%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    //https://pdocs.kauailabs.com/navx-micro/examples/straight-line-driving/
}

