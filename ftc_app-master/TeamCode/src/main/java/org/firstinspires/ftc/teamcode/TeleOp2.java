package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static android.os.SystemClock.sleep;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

@TeleOp(name = "TeleOp")
@Disabled
public class TeleOp2 extends OpMode {

    robotBase robot                     = new robotBase();
    private ElapsedTime runtime         = new ElapsedTime();

    int direction = -1;
    double boost = 1.35;
    int targetPos= -2000;

    double leftPower = 0.0;
    double rightPower = 0.0;
    boolean found = false;
    //private Servo tester = null;
    @Override
    public void init() {
        robot.init(hardwareMap);
        robot.ADM.setMode(STOP_AND_RESET_ENCODER);
        robot.ADM.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.leftDrive.setMode(RUN_USING_ENCODER);
        robot.rightDrive.setMode(RUN_USING_ENCODER);

        robot.inVertical.setMode(STOP_AND_RESET_ENCODER);
        robot.inVertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.inVertical.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        runtime.reset();

    }

    public void start() {
    if(robot.inVertical.getCurrentPosition()>targetPos) {
            while ((robot.inVertical.getCurrentPosition() > targetPos)) {
                telemetry.addData("EncoderPos", robot.inVertical.getCurrentPosition());
                telemetry.update();
                robot.inVertical.setPower(0.03);
            }
        }

        robot.inVertical.setPower(0);
    }

    @Override
    public void loop() {
        if(direction == 1) {
            leftPower = robot.getWheelPower(gamepad1.left_stick_y);
            rightPower = robot.getWheelPower(gamepad1.right_stick_y);
        }
        else if(direction == -1){
            rightPower = robot.getWheelPower(gamepad1.left_stick_y);
            leftPower = robot.getWheelPower(gamepad1.right_stick_y);
        }
        if(gamepad1.right_trigger > .25) {
            robot.leftDrive.setPower((leftPower * direction * boost));
            robot.rightDrive.setPower((rightPower * direction * boost));
        }
        else{
            robot.leftDrive.setPower((leftPower * direction));
            robot.rightDrive.setPower((rightPower * direction));
        }


        if(gamepad1.a){
            robot.ADM.setTargetPosition((int)(-robot.LEAD_SCREW_TURNS * robot.COUNTS_PER_MOTOR_REV_rev)+200);
            robot.ADM.setPower(-.5);
        }
        if(gamepad1.y){
            robot.ADM.setTargetPosition(-50);
            robot.ADM.setPower(.5);
        }

        if(gamepad1.x){
            robot.traverse.setPosition(robot.maxTraverse);
        }
        if(gamepad1.b){
            robot.traverse.setPosition(robot.minTraverse);
        }
        if(gamepad1.right_bumper){
            robot.traverse.setPosition(robot.midTraverseRight);
        }
        if(gamepad1.left_bumper){
            robot.traverse.setPosition(robot.midTraverseLeft);
        }

        if(gamepad1.dpad_up) {
            direction *= -1;
            sleep(500);
        }
        if(gamepad1.dpad_down) {
            robot.marker.setPosition(robot.markerMid);
            sleep(250);
        }

        if(gamepad1.dpad_left){
            double pos = robot.traverse.getPosition() + .02;
            if(pos < robot.maxTraverse) {
                robot.traverse.setPosition(pos);
                sleep(250);
            }
        }

        if(gamepad1.dpad_right){
            double pos = robot.traverse.getPosition() - .02;
            if(pos > robot.minTraverse) {
                robot.traverse.setPosition(pos);
                sleep(250);
            }
        }

        //------------------------------------------------------------------------------------------

        /// make arm move until reached hall effect sensor, then stop


        /*while (robot.vertHall.getState() && gamepad2.x) {
            robot.inVertical.setPower(-0.03);
            found = true;
        }
        if(found && !robot.vertHall.getState()){
            sleep(340);
            robot.inVertical.setPower(0);
            found = false;
        }*/



        robot.inVertical.setPower(-gamepad2.right_stick_y * 0.075);

        //Set motor power to stick input, directionally scaled
        robot.inHorizontal.setPower(gamepad2.left_stick_y);

        if(gamepad2.dpad_up)
            robot.intake.setPower(1);
        else if(gamepad2.dpad_down)
            robot.intake.setPower(-1);
        else
            robot.intake.setPower(0);


        if(gamepad2.a)
            robot.intakeGate.setPosition(1.0);
        else
            robot.intakeGate.setPosition(-1.0);

        telemetry.addData("Time", Math.round(runtime.seconds() - 8));
        telemetry.update();
    }
}