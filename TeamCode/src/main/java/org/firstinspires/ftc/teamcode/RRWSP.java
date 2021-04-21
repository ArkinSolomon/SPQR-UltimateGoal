package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * This OpMode starts from the right-most red starting point against the wall, with the wobble goal
 * loaded on the intake drop.
 *
 * Autonomous: Red Right Wobble-Goal Shoot Park
 *
 * @author Arkin Solomon
 */
@Autonomous(name="RRWSP", group="Competition")
public class RRWSP extends SPQRLinearOpMode {

    @Override
    public void runOpMode(){
        robot.init(hardwareMap);
        waitForStart();
        Rings rings = waitForRings(5000);
        telemetry.addData("Rings", rings);
        drive(1800, 1);
//        switch (rings){
//            case NONE:
//                drive(1800, 1);
//                break;
//            case ONE:
//
//                break;
//            case FOUR:
//
//                break;
//        }
    }
}
