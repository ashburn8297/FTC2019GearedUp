package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import net.frogbots.ftcopmodetunercommon.opmode.TunableOpMode;

import org.firstinspires.ftc.teamcode.robotBase;

import static android.os.SystemClock.sleep;

@TeleOp(name = "TeleOp Test")
//@Disabled
public class driveBaseTesting extends TunableOpMode {

    robotBase robot                     = new robotBase();
    private ElapsedTime runtime         = new ElapsedTime();

    double leftPower = 0.0;
    double rightPower = 0.0;
    double servoPower = 0.0;

    boolean aDown = false;
    boolean bDown = false;
    boolean xDown = false;
    boolean yDown = false;
    boolean rbDown = false;

    @Override
    public void init() {
        robot.init(hardwareMap);
        //Home servo so that Lim can reach ADM
        robot.traverse.setPosition(robot.minTraverse);
        sleep(3000);
        //Home ADM
        while(robot.admLim.getState() == false) {
            robot.ADM.setPower(-.5);
        }
        robot.ADM.setPower(.1);
        robot.ADM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.ADM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.ADM.setTargetPosition(0);
    }

    @Override
    public void loop() {

        leftPower = robot.getWheelPower(-gamepad1.left_stick_y);
        rightPower = robot.getWheelPower(-gamepad1.right_stick_y);

        robot.leftDrive.setPower(leftPower);
        robot.rightDrive.setPower(rightPower);

        yDown   = gamepad1.y;
        aDown   = gamepad1.a;
        xDown   = gamepad1.x;
        bDown   = gamepad1.b;
        rbDown  = gamepad1.right_bumper;

        if(yDown){
            robot.ADM.setTargetPosition((int)(robot.LEAD_SCREW_TURNS * robot.COUNTS_PER_MOTOR_REV));
            robot.ADM.setPower(.75);
            if(robot.ADM.getCurrentPosition() > robot.ADM.getTargetPosition() - 550) {
                yDown = false;
                robot.ADM.setPower(0);
            }
        }

        if(aDown){
            robot.ADM.setTargetPosition(0);
            robot.ADM.setPower(.75);
            if(robot.ADM.getCurrentPosition() < 550) {
                aDown = false;
                robot.ADM.setPower(0);
            }
        }

        if(xDown){
            robot.traverse.setPosition(robot.maxTraverse);
            xDown = false;
        }

        if(bDown){
            robot.traverse.setPosition(robot.minTraverse);
            bDown = false;
        }

        if(rbDown) {
            robot.traverse.setPosition(robot.midTraverse);
            rbDown = false;
        }
    }
}
