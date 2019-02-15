package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
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

public class robotBaseInitialize
{
    /* Public OpMode members. */

    public DcMotor ADM                      = null; //ascent_descent

    public Servo traverse                   = null; //ADM_servo
    public Servo intakeGate                 = null; //intake_pitch

    public DigitalChannel hall              = null; //
    public DigitalChannel vertHall          = null; //vertHall
    IntegratingGyroscope gyro;

    /* local OpMode members. */
    HardwareMap hwMap                       = null;
    private ElapsedTime period              = new ElapsedTime();

    //Items for encoders
    public static final double  COUNTS_PER_MOTOR_REV_rev      = 560.0;
    public static final double  DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    public static final double  WHEEL_DIAMETER_INCHES = 4.0;    // For figuring circumference

    public static final double  COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV_rev * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    public static final double  DRIVE_SPEED = 0.24; //.23
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

    public robotBaseInitialize(){

    }
    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        ADM = hwMap.get(DcMotor.class, "ascent_descent"); //Control Ascent Descent Module (ADM)

        // Initialize direction of motors
         ADM.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        ADM.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
       ADM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //inVertical.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        hall = hwMap.get(DigitalChannel.class, "hall");
        vertHall = hwMap.get(DigitalChannel.class, "vertHall");

        traverse = hwMap.get(Servo.class, "ADM_servo");
        traverse.setDirection(Servo.Direction.FORWARD);

        intakeGate = hwMap.get(Servo.class, "intake_pitch");
        intakeGate.setDirection(Servo.Direction.FORWARD);
    }
    //Auto Methods

    String formatDegrees(double degrees){
        return String.format("%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    //https://pdocs.kauailabs.com/navx-micro/examples/straight-line-driving/
}

