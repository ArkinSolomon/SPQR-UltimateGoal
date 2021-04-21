package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 *
 * This is the main opmode used for Lightbot in the driver-controlled period. Start anywhere.
 *
 * TeleOp: BOTH ALLIANCES
 *
 * Driver controls:
 *  - Right bumper: Strafe right.
 *  - Left bumper: Strafe left.
 *  - Right stick Y: Speed of the right drive motors of the robot.
 *  - Left stick Y: Speed of the left drive motors of the robot.
 *  - Button 'a': Reverse direction.
 *  - Button 'b': Toggle sniper mode.
 *
 * @author Arkin Solomon
 */
@TeleOp(name="LB Main OpMode", group="Lightbot")
public class LightbotMainOpMode extends OpMode {

    //Robot hardware instance reference
    private HardwareLightbot robot;

    //Speed of the robot
    private double speed = 1.0;

    //Gamepad inputs
    private Input gamepad1Input, gamepad2Input;

    @Override
    public void init(){

        //Initialize hardware
        robot = new HardwareLightbot(hardwareMap);

        //Rewrite gamepads
        gamepad1Input = new Input(gamepad1);
        gamepad2Input = new Input(gamepad2);
    }

    @Override
    public void loop(){

        //Update inputs
        gamepad1Input.updateAll();
        gamepad2Input.updateAll();

        /* Driver controls */

        //Left and right strafing
        if (gamepad1Input.left_bumper.isDown){
            if (gamepad1Input.right_bumper.isDown) return;
            robot.strafe(Dir.RIGHT, speed);
        }
        if (gamepad1Input.right_bumper.isDown) {
            if (gamepad1Input.left_bumper.isDown) return;
            robot.strafe(Dir.LEFT, speed);
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
    }
}
