package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static android.os.SystemClock.sleep;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

@TeleOp(name = "TeleOpREDO")
//Disabled
public class teleopRedo extends OpMode {

    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private Servo traverse  = null; //ADM_servo
    private DcMotor ADM = null; //ascent_descent
    private Servo marker = null; //team_marker
    private Servo intakeGate = null; //intake_pitch
    private CRServo intake = null; //intake
    public DcMotor inVertical               = null; //intake_veritcal
    public DcMotor inHorizontal             = null; //intake_horizontal
    public DigitalChannel vertHall          = null; //vertHall

    int direction = -1;
    double leftPower = 0.0;
    double rightPower = 0.0;
    double boost = 1.35;
    int targetPos= -2400;
    boolean found = false;

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
    @Override
    public void init() {
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        ADM = hardwareMap.get(DcMotor.class, "ascent_descent"); //Control Ascent Descent Module (ADM)
        ADM.setMode(STOP_AND_RESET_ENCODER);
        ADM.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        traverse = hardwareMap.get(Servo.class, "ADM_servo");
        traverse.setDirection(Servo.Direction.FORWARD);

        marker = hardwareMap.get(Servo.class, "team_marker");
        marker.setDirection(Servo.Direction.FORWARD);

        intakeGate = hardwareMap.get(Servo.class, "intake_pitch");
        intakeGate.setDirection(Servo.Direction.FORWARD);

        inVertical = hardwareMap.get(DcMotor.class, "intake_vertical");
        inHorizontal = hardwareMap.get(DcMotor.class, "intake_horizontal");
        inHorizontal.setDirection(DcMotor.Direction.REVERSE);
        inVertical.setDirection(DcMotor.Direction.REVERSE);
        inVertical.setMode(STOP_AND_RESET_ENCODER);
        inVertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        inHorizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        inVertical.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        intake = hardwareMap.get(CRServo.class, "intake");
        intake.setDirection(CRServo.Direction.FORWARD);

        vertHall = hardwareMap.get(DigitalChannel.class, "vertHall");


    }

    public void start() {
        if(inVertical.getCurrentPosition()>targetPos) {
            while ((inVertical.getCurrentPosition() > targetPos)) {
                telemetry.addData("EncoderPos", inVertical.getCurrentPosition());
                telemetry.update();
                inVertical.setPower(0.03);
            }
        }

        inVertical.setPower(0);
        //traverse.setPosition(minTraverse);

    }

    @Override
    public void loop() {

        if(gamepad1.dpad_up) {
            direction *= -1;
            sleep(500);
        }

        if(direction == 1) {
            leftPower = getWheelPower(gamepad1.left_stick_y);
            rightPower = getWheelPower(gamepad1.right_stick_y);
        }
        else if(direction == -1){
            rightPower = getWheelPower(gamepad1.left_stick_y);
            leftPower = getWheelPower(gamepad1.right_stick_y);
        }
        if(gamepad1.right_trigger > .25) {
            leftDrive.setPower((leftPower * direction * boost));
            rightDrive.setPower((rightPower * direction * boost));
        }
        else{
            leftDrive.setPower((leftPower * direction));
            rightDrive.setPower((rightPower * direction));
        }

        if(gamepad1.a){
           ADM.setTargetPosition((int)(-LEAD_SCREW_TURNS * COUNTS_PER_MOTOR_REV_rev)+200);
            ADM.setPower(-.5);
        }
        if(gamepad1.y){
            ADM.setTargetPosition(-50);
            ADM.setPower(.5);
        }

        if(gamepad1.x){
            traverse.setPosition(maxTraverse);
        }
        if(gamepad1.b){
            traverse.setPosition(minTraverse);
        }
        if(gamepad1.right_bumper){
            traverse.setPosition(midTraverseRight);
        }
        if(gamepad1.left_bumper){
            traverse.setPosition(midTraverseLeft);
        }

        if(gamepad1.dpad_left){
            double pos = traverse.getPosition() + .02;
            if(pos < maxTraverse) {
                traverse.setPosition(pos);
                sleep(250);
            }
        }

        if(gamepad1.dpad_right){
            double pos = traverse.getPosition() - .02;
            if(pos > minTraverse) {
                traverse.setPosition(pos);
                sleep(250);
            }
        }

        if(gamepad1.dpad_down) {
            marker.setPosition(markerMid);
            sleep(250);
        }

        if(gamepad2.a)
            intakeGate.setPosition(1.0);
        else
            intakeGate.setPosition(-1.0);

        if(gamepad2.right_trigger > 0 || gamepad2.right_bumper)
            intake.setPower(.8);
        else if(gamepad2.left_trigger > 0 || gamepad2.left_bumper)
            intake.setPower(-.8);
        else
            intake.setPower(0.0);

        /*
        while (vertHall.getState() && gamepad2.x) {
            inVertical.setPower(-0.03);
            found = true;
        }
        if(found && !vertHall.getState()){
            sleep(340);
            inVertical.setPower(0);
            found = false;
        }
        */



        inVertical.setPower(-gamepad2.right_stick_y * 0.075);
        inHorizontal.setPower(gamepad2.left_stick_y);

        /*telemetry.addData("Arm", inVertical.getCurrentPosition());
        telemetry.addData("VertHAll", vertHall.getState());
        //Set motor power to stick input, directionally scale

        telemetry.addData("Right Stick",gamepad1.right_stick_y);
        telemetry.addData("Left Stick",gamepad1.left_stick_y);
        telemetry.addData("Right Drive",rightDrive.getCurrentPosition());
        telemetry.addData("Left Drive",leftDrive.getCurrentPosition());*/

    }
    public static double getWheelPower(double in){
        in *= .27;
        return in;
    }
}