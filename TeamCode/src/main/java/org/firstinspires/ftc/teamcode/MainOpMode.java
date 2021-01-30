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

    @Override
    public void init(){

        //Initialize hardware
        robot.init(hardwareMap);
        robot.dropIntake();

        //Rewrite gamepads
        gamepad1 = new Input(gamepad1);
        gamepad2 = new Input(gamepad2);
    }

    @Override
    public void loop(){

        /* Driver controls */

        //Left and right strafing
        if (((Input) gamepad1).left_bumper.isDown){
            if (((Input) gamepad1).right_bumper.isDown) return;
            robot.strafe(Dir.LEFT, speed);
        }
        if (((Input) gamepad1).right_bumper.isDown) {
            if (((Input) gamepad1).left_bumper.isDown) return;
            robot.strafe(Dir.RIGHT, speed);
        }

        //Reverse directions
        if (((Input) gamepad1).a.down){
            speed *= -1;
        }

        //Tank movement
        double right = -((Input) gamepad1).right_stick_y.value * speed;
        double left = -((Input) gamepad1).left_stick_y.value * speed;
        if (speed < 0){
            double l = left;
            left = right;
            right = l;
        }

        //Only tank move if not strafing
        if (!((Input) gamepad1).left_bumper.isDown && !((Input) gamepad1).right_bumper.isDown){
            robot.tank(right, left);
        }

        //Sniper mode
        if (((Input) gamepad1).b.down) {
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
        if (((Input) gamepad2).dpad_down.down){
            robot.dropIntake();
        }

        //Raise lower intake
        if (((Input) gamepad2).dpad_up.down){
            robot.raiseIntake();
        }

        //Control intake
        if (((Input) gamepad2).left_trigger.isDown){
            robot.startIntake();
        }else if (((Input) gamepad2).dpad_right.down){
            robot.spitIntake();
        }else{
            robot.stopIntake();
        }

        //Control cannon
        if (((Input) gamepad2).dpad_left.down){
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
