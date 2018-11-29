package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import static android.os.SystemClock.sleep;

@TeleOp(name = "TeleOp Test")
//@Disabled
public class driveBaseTesting extends OpMode {

    robotBase robot                     = new robotBase();
    private ElapsedTime runtime         = new ElapsedTime();

    int direction = -1;

    double leftPower = 0.0;
    double rightPower = 0.0;

    boolean foundState = false;

    boolean yDown = false;
    boolean aDown = false;
    boolean bDown = false;
    boolean rbDown = false;
    boolean lbDown = false;
    boolean xDown = false;
    boolean dpadUp = false;
    boolean dpadLeft = false;
    boolean dpadRight = false;
    boolean yDown2 = false;
    boolean xDown2 = false;
    boolean aDown2 = false;

    @Override
    public void init_loop(){

    }

    @Override
    public void init() {
        robot.init(hardwareMap);
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


        yDown = gamepad1.y;
        aDown = gamepad1.a;
        xDown = gamepad1.x;
        bDown = gamepad1.b;
        rbDown = gamepad1.right_bumper;
        lbDown = gamepad1.left_bumper;
        dpadUp = gamepad1.dpad_up;
        dpadLeft = gamepad1.dpad_left;
        dpadRight = gamepad1.dpad_right;
        yDown2 = gamepad2.y;
        aDown2 = gamepad2.a;

        if(yDown){
            robot.ADM.setTargetPosition((int)(robot.LEAD_SCREW_TURNS * robot.COUNTS_PER_MOTOR_REV_neverest));
            robot.ADM.setPower(1);
            yDown = false;
        }

        if(aDown){
            robot.ADM.setTargetPosition(100);
            robot.ADM.setPower(-1);
            aDown= false;
        }

        if(xDown){
            robot.traverse.setPosition(robot.maxTraverse);
            xDown = false;
        }

        if(bDown){
            robot.traverse.setPosition(robot.minTraverse);
            bDown = false;
        }

        if(rbDown){
            robot.traverse.setPosition(robot.midTraverseRight);
            rbDown = false;
        }

        if(lbDown){
            robot.traverse.setPosition(robot.midTraverseLeft);
            lbDown = false;
        }

        if(dpadUp) {
            direction *= -1;
            sleep(500);
            dpadUp = false;
        }

        if(dpadLeft){
            double pos = robot.traverse.getPosition() + .02;
            if(pos < robot.maxTraverse) {
                robot.traverse.setPosition(pos);
                sleep(250);
            }
            dpadLeft = false;
        }

        if(dpadRight){
            double pos = robot.traverse.getPosition() - .02;
            if(pos > robot.minTraverse) {
                robot.traverse.setPosition(pos);
                sleep(250);
            }
            dpadRight = false;
        }

        if(yDown2){
            robot.inVertical.setTargetPosition(-70);
            robot.inVertical.setPower(-.25);
            yDown2 = false;
        }

        if(aDown2){
            robot.inVertical.setTargetPosition(-5);
            robot.inVertical.setPower(.25);
            aDown2 = false;
        }
        telemetry.addData("Pos", robot.inVertical.getCurrentPosition());
        telemetry.update();
    }
}
