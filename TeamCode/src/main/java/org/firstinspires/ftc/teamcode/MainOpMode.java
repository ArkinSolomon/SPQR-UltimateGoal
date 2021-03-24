package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

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
 *  - Button 'a': Toggle hand
 *  - Button 'b': Toggle stabilization
 *
 * @author Arkin Solomon
 */
@TeleOp(name="Main OpMode", group="Competition")
public class MainOpMode extends OpMode {

    private HardwareSPQR robot = new HardwareSPQR(telemetry);

    //Speed of the robot
    private double speed = -1.0;

    //If the hand is closed
    private boolean isHandClosed = true;

    //Gamepad inputs
    private Input gamepad1Input, gamepad2Input;

    //If the arm is being stabilized
    private boolean isHandStabilizationActive = false;

    @Override
    public void init(){

        //Initialize hardware
        robot.init(hardwareMap);
//        robot.dropIntake();
        robot.armMotor.setTargetPosition(430);
        robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.armMotor.setPower(0.5);
        robot.wristServo.setPosition(1.0);

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
        }else if (gamepad2Input.dpad_right.isDown){
            robot.spitIntake();
        }else{
            robot.stopIntake();
        }

        //Control cannon
        if (gamepad2Input.right_trigger.isDown){
            robot.setCannonSpeed(1);
        }else if (gamepad2Input.dpad_right.isDown){
            robot.setCannonSpeed(-.1);
        }else{
            robot.setCannonSpeed(0);
        }

        //Grab hand
        if (gamepad2Input.a.down){
            if (isHandClosed){
                robot.handRelease();
                isHandClosed = false;
            }else{
                robot.handGrab();
                isHandClosed = true;
            }
        }

        //Move arm
        int armEncoder = robot.armMotor.getTargetPosition() - (int) Math.round(gamepad2Input.right_stick_y.value * 10);
        telemetry.addData("g2RsYV", gamepad2Input.right_stick_y.value);
        telemetry.addData("arm pos", armEncoder);
        telemetry.addData("arm target", robot.armMotor.getTargetPosition());
        robot.armMotor.setTargetPosition(armEncoder);
        robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.armMotor.setPower(0.5);

        //Stabilize hand
        if (isHandStabilizationActive) {

            double stablePos = robot.getWristServoPosition(robot.armMotor.getCurrentPosition());

            robot.wristServo.setPosition(stablePos);

            telemetry.addData("StablestablePos", stablePos);

            if (gamepad2Input.b.down){
                isHandStabilizationActive = false;
            }
        }else{

            //Manual hand control
            double newPos = robot.wristServo.getPosition() - gamepad2Input.left_stick_y.value / 100;
            if (newPos > 1) {
                newPos = 1;
            } else if (newPos < -1) {
                newPos = -1;
            }
            telemetry.addData("g2LsYV", gamepad2Input.left_stick_y.value);
            telemetry.addData("servo pos", newPos);
            robot.wristServo.setPosition(newPos);

            if (gamepad2Input.b.down){
                isHandStabilizationActive = true;
            }
        }

        //Reset encoder position
        if (gamepad2Input.back.down){
            robot.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.armMotor.setTargetPosition(0);
            robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.armMotor.setPower(0.5);
        }

        //Update telemetry
        telemetry.update();
    }
}
