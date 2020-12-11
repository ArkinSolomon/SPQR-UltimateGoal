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
 *  - Button 'b': Toggle sniper mode
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
        this.robot.init(hardwareMap);
    }

    @Override
    public void loop() {

        /* Left and right strafing movement */
        if (gamepad1.left_bumper){
            if (gamepad1.right_bumper) return;
            this.robot.strafe(Dir.LEFT, this.speed);
        }
        if (gamepad1.right_bumper) {
            if (gamepad1.left_bumper) return;
            this.robot.strafe(Dir.RIGHT, this.speed);
        }

        /* Tank movement */
        double right = -gamepad1.right_stick_y * this.speed;
        double left = -gamepad1.left_stick_y * this.speed;
        if (this.speed < 0){
            double l = left;
            left = right;
            right = l;
        }

        //Only tank move if not strafing
        if (!gamepad1.left_bumper && !gamepad1.right_bumper){
            this.robot.tank(right, left);
        }

        /* Reverse direction */
        if (gamepad1.a) {
            if (this.gamepad1_aPressed) return;
            this.gamepad1_aPressed = true;
            this.speed *= -1;
        }
        if (!gamepad1.a){
            this.gamepad1_aPressed = false;
        }

        /* Sniper mode */
        if (gamepad1.b) {
            if (this.gamepad1_bPressed) return;
            this.gamepad1_bPressed = true;
            if (this.speed > 0){
                if (this.speed > 0.5){
                    this.speed = 0.5;
                }else{
                    this.speed = 1.0;
                }
            }else{
                if (this.speed < -0.5){
                    this.speed = -0.5;
                }else{
                    this.speed = -1.0;
                }
            }
        }
        if (!gamepad1.b){
            this.gamepad1_bPressed = false;
        }
        telemetry.update();
    }
}
