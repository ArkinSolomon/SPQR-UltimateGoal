package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * TeleOp: DEVELOPMENT
 *
 * This opmode is used for development and should NOT be used in game.
 *
 * @author Arkin Solomon
 */
@Autonomous(name="VuforiaTestOpMode", group="Developmet")
public class VuforiaTestOpMode extends SPQRLinearOpMode {

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        waitForStart();
        Rings rings = waitForRings(5000);
        telemetry.addData("Rings", rings.toString());
        telemetry.update();
        while (opModeIsActive()){}
    }
}
