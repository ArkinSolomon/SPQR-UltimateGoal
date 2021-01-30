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
 *  - Dpad down: Lower intake.
 *  - Dpad up: Raise intake.
 *  - Left trigger: Start intake.
 *  - Dpad left: Spit intake.
 *  - Dpad right: Suck cannon.
 *  - Right trigger: Shoot cannon.
 *
 * @author Arkin Solomon
 */
@TeleOp(name="Main OpMode", group="Competition")
public class MainOpMode extends OpMode {

    private HardwareSPQR robot = new HardwareSPQR(telemetry);

    //Speed of the robot
    private double speed = 1.0;

    //Gamepad inputs
    private Input gamepad1Input, gamepad2Input;

    @Override
    public void init(){

        //Initialize hardware
        robot.init(hardwareMap);
        robot.dropIntake();

        //Rewrite gamepads
        gamepad1Input = new Input(gamepad1);
        gamepad2Input = new Input(gamepad2);
    }

    @Override
    public void loop(){

        /* Driver controls */

        //Left and right strafing
        if (gamepad1Input.left_bumper.isDown){
            if (gamepad1Input.right_bumper.isDown) return;
            robot.strafe(Dir.LEFT, speed);
        }
        if (gamepad1Input.right_bumper.isDown) {
            if (gamepad1Input.left_bumper.isDown) return;
            robot.strafe(Dir.RIGHT, speed);
        }

        //Reverse directions
        if (gamepad1Input.a.down){
            speed *= -1;
        }

        //Tank movement
        double right = -gamepad1Input.right_stick_y.value * speed;
        double left = -gamepad1Input.left_stick_y.value * speed;
        if (speed < 0){
            double l = left;
            left = right;
            right = l;
        }

        //Only tank move if not strafing
        if (!gamepad1Input.left_bumper.isDown && !gamepad1Input.right_bumper.isDown){
            robot.tank(right, left);
        }

        //Sniper mode
        if (gamepad1Input.b.down) {
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

        /* Gunner controls */

        //Drop lower intake
        if (gamepad2Input.dpad_down.down){
            robot.dropIntake();
        }

        //Raise lower intake
        if (gamepad2Input.dpad_up.down){
            robot.raiseIntake();
        }

        //Control intake
        if (gamepad2Input.left_trigger.isDown){
            robot.startIntake();
        }else if (gamepad2Input.dpad_right.down){
            robot.spitIntake();
        }else{
            robot.stopIntake();
        }

        //Control cannon
        if (gamepad2Input.dpad_left.down){
            robot.setCannonSpeed(1);
        }else if (gamepad2.dpad_right){
            robot.setCannonSpeed(-.1);
        }else{
            robot.setCannonSpeed(0);
        }

        //Update telemetry
        telemetry.update();

    }
}
