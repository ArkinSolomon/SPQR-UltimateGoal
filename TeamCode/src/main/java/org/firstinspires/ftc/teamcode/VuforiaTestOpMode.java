package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * TeleOp: DEVELOPMENT
 *
 * This opmode is used for development and should NOT be used in game.
 *
 * @author Arkin Solomon
 */
@TeleOp(name="VuforiaTestOpMode", group="Developmet")
public class VuforiaTestOpMode extends OpMode {

    private HardwareSPQR robot = new HardwareSPQR(telemetry, true);

    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {
        robot.updateRobotPosition();
        robot.updateObjectDetection();
        telemetry.update();
    }
}
