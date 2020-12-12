package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * TeleOp: BOTH ALLIANCES
 *
 * Start anywhere. This is the main opmode used in the driver-controlled period.
 *
 * Driver controls:
 *  - Right bumper: Strafe right.
 *  - Left bumper: Strafe left.
 *  - Right stick Y: Speed of the right drive motors of the robot.
 *  - Left stick Y: Speed of the left drive motors of the robot.
 *  - Button 'a': Reverse direction.
 *  - Button 'b': Toggle sniper mode.
 *
 * Gunner controls:
 *  - Left trigger: Start intake.
 *  - Dpad left: Spit intake.
 *
 * @author Arkin Solomon
 */
@TeleOp(name="Main OpMode", group="Competition")
public class MainOpMode extends OpMode {

    private HardwareSPQR robot = new HardwareSPQR(telemetry);

    //Speed of the robot
    private double speed = 1.0;

    //Prevent detecting multiple clicks
    private boolean gamepad1_aPressed = false;
    private boolean gamepad1_bPressed = false;

    @Override
    public void init() {

        //Initialize hardware
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {

        /* Driver controls */

        //Left and right strafing
        if (gamepad1.left_bumper){
            if (gamepad1.right_bumper) return;
            robot.strafe(Dir.LEFT, speed);
        }
        if (gamepad1.right_bumper) {
            if (gamepad1.left_bumper) return;
            robot.strafe(Dir.RIGHT, speed);
        }

        //Tank movement
        double right = -gamepad1.right_stick_y * speed;
        double left = -gamepad1.left_stick_y * speed;
        if (speed < 0){
            double l = left;
            left = right;
            right = l;
        }

        //Only tank move if not strafing
        if (!gamepad1.left_bumper && !gamepad1.right_bumper){
            robot.tank(right, left);
        }

        //Reverse directions
        if (gamepad1.a) {
            if (gamepad1_aPressed) return;
            gamepad1_aPressed = true;
            speed *= -1;
        }
        if (!gamepad1.a){
            gamepad1_aPressed = false;
        }

        //Sniper mode
        if (gamepad1.b) {
            if (gamepad1_bPressed) return;
            gamepad1_bPressed = true;
            if (speed > 0){
                if (speed > 0.5){
                    speed = 0.5;
                }else{
                    speed = 1.0;
                }
            }else{
                if (speed < -0.5){
                    speed = -0.5;
                }else{
                    speed = -1.0;
                }
            }
        }
        if (!gamepad1.b){
            gamepad1_bPressed = false;
        }

        /* Gunner controls */

        //Drop lower intake
        if (gamepad2.a){
            robot.dropIntake();
        }

        //Raise lower intake
        if (gamepad2.b){
            robot.raiseIntake();
        }

        //Start intake
        if (gamepad2.left_trigger > 0.25){
            robot.lowIntakeMotor.setPower(1);
        }else if (gamepad2.dpad_left){
            robot.lowIntakeMotor.setPower(-1);
        }else{
            robot.lowIntakeMotor.setPower(0);
        }

        //Update telemetry
        telemetry.update();
    }
}
