package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import static android.os.SystemClock.sleep;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

@TeleOp(name = "TeleOp")
//Disabled
public class teleOp extends OpMode {
    robotBase robot                     = new robotBase();
    private ElapsedTime runtime         = new ElapsedTime();

    int direction = -1;

    double leftPower = 0.0;
    double rightPower = 0.0;

    @Override
    public void init() {
        robot.init(hardwareMap);
        robot.inVertical.setMode(STOP_AND_RESET_ENCODER);
        robot.inVertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.ADM.setMode(STOP_AND_RESET_ENCODER);
        robot.ADM.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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
        robot.leftDrive.setPower((leftPower * direction));
        robot.rightDrive.setPower((rightPower * direction));


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
        robot.inVertical.setMode(RUN_WITHOUT_ENCODER);
        robot.inVertical.setPower(gamepad2.right_stick_y * 0.45);

        //Set motor power to stick input, directionally scaled
        robot.inHorizontal.setPower(gamepad2.left_stick_y);

        //Control Intake
        if(gamepad2.right_trigger > 0)
            robot.intake.setPower(-1.0);
        else if(gamepad2.left_trigger > 0)
            robot.intake.setPower(1.0);
        else
            robot.intake.setPower(0.0);

        if(gamepad2.dpad_left) {
            robot.intakePitch.setPosition(robot.boxFlat);
            sleep(500);
        }
        if(gamepad2.dpad_right) {
            robot.intakePitch.setPosition(robot.boxDump);
            sleep(500);
        }
        if(gamepad2.dpad_up) {
            robot.intakePitch.setPosition(robot.boxStowed);
            sleep(500);
        }
        if(gamepad2.dpad_down){
            robot.intakePitch.setPosition(robot.boxIntake);
            sleep(500);
        }

        telemetry.addData("Pos", robot.inVertical.getCurrentPosition());
        telemetry.addData("Out", robot.inHorizontal.getCurrentPosition());
        telemetry.addData("Marker", robot.marker.getPosition());
        telemetry.addData("Encoder REV", robot.ADM.getCurrentPosition());
        telemetry.addData("Servo Pitch", robot.intakePitch.getPosition());
        telemetry.update();
    }
}
