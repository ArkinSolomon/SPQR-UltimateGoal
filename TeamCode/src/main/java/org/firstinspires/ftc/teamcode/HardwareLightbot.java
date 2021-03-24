package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/*
 * Hardware configurations
 *
 * Motor channel H1-0 (NeveRest 40 Gearmotor):        Right front drive motor:   "right_front_drive"
 * Motor channel H1-1 (NeveRest 40 Gearmotor):        Left front drive motor:    "left_front_drive"
 * Motor channel H1-2 (NeveRest 40 Gearmotor):        Right back drive motor:    "right_back_drive"
 * Motor channel H1-3 (NeveRest 40 Gearmotor):        Left back drive motor:     "left_back_drive"
 */

/**
 * This hardware class is for Lightbot, the new robot. It declares and defines all hardware
 * components on the robot, as well as provides control abstractions.
 *
 * @author Arkin Solomon
 */
public class HardwareLightbot {

    //True if robot is initialized
    private boolean robotIsInitialized = false;

    //Hardware map
    private HardwareMap hwMap = null;

    //Sound player instance
    public SoundPlayer soundPlayer;

    //Declare hardware
    public DcMotor rightFrontDrive = null;
    public DcMotor leftFrontDrive = null;
    public DcMotor rightBackDrive = null;
    public DcMotor leftBackDrive = null;

    /**
     * Class constructor. Returns a new instance of HardwareLightbot. The constructor also defines
     * all declared hardware components on execution.
     *
     * @param map The hardware map of the class instance of the callee to be assigned to the
     *            instance of hardware.
     */
    public HardwareLightbot(HardwareMap map){

        //Initialize hardware map
        hwMap = map;

        /* Initialize motors*/

        //Define motors
        if (DevVars.enableDriveMotors){
            rightFrontDrive = hwMap.get(DcMotor.class, "right_front_drive");
            leftFrontDrive = hwMap.get(DcMotor.class, "left_front_drive");
            rightBackDrive = hwMap.get(DcMotor.class, "right_back_drive");
            leftBackDrive = hwMap.get(DcMotor.class, "left_back_drive");
        }

        //Set motor direction
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);

        //Set drive motors to brake
        setDriveZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Set all motor power to zero
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);

        //Get sound player
        soundPlayer = SoundPlayer.getInstance();

        //Set initialization state to true
        this.robotIsInitialized = true;
    }

    /**
     * This method takes two powers and sets the left motors to a given power and a right motors to
     * a given power respectively.
     *
     * @param left A double between -1.0 and 1.0 which determines the speed that the left motors
     *              will be set to.
     * @param right A double between -1.0 and 1.0 which determines the speed that the right motors
     *              will be set to.
     */
    public void tank(double left, double right){
        if (!robotIsInitialized) throw new Error("Robot not initialized");
        leftFrontDrive.setPower(left);
        leftBackDrive.setPower(left);
        rightFrontDrive.setPower(right);
        rightBackDrive.setPower(right);
        if (left != 0 || right != 0){
//            soundPlayer.startPlaying(hwMap.appContext, pacmanChomp);
        }else if (left == 0 && right == 0){
            soundPlayer.stopPlayingAll();
        }
    }

    /**
     * This method sets the powers of the motors in a way that will strafe the robot in a given
     * direction.
     *
     * @param direction A Dir enumeration which states which direction the robot will strafe.
     * @param power     A double between -1.0 and 1.0 which represents the speed at which the robot
     *                  is to strafe.
     */
    public void strafe(Dir direction, double power){
        if (!robotIsInitialized) throw new Error("Robot not initialized");
        leftFrontDrive.setPower((direction == Dir.LEFT) ? -power : power);
        leftBackDrive.setPower((direction == Dir.LEFT) ? power : -power);
        rightFrontDrive.setPower((direction == Dir.LEFT) ? power : -power);
        rightBackDrive.setPower((direction == Dir.LEFT) ? -power : power);
        if (power != 0){
//            soundPlayer.startPlaying(hwMap.appContext, pacmanChomp);
        }else{
            soundPlayer.stopPlayingAll();
        }
    }

    /**
     * This method makes the robot go at full speed forward by setting all of the motor powers to
     * 1.0.
     */
    public void forward() {
        if (!robotIsInitialized) throw new Error("Robot not initialized");
        this.setPowers(1.0);
//        soundPlayer.startPlaying(hwMap.appContext, pacmanChomp);
    }

    /**
     * This method stops the movement of the drive motors. If the motors are set to brake on zero
     * power the robot will stop in place.
     */
    public void stopMoving(){
        if (!robotIsInitialized) throw new Error("Robot not initialized");
        this.setPowers(0);
        soundPlayer.stopPlayingAll();
    }

    /**
     * This method makes the robot go at full speed backward by setting all of the motor powers to
     * -1.0.
     */
    public void backward(){
        if (!robotIsInitialized) throw new Error("Robot not initialized");
        this.setPowers(-1.0);
//        soundPlayer.startPlaying(hwMap.appContext, pacmanChomp);
    }

    /**
     * This method sets all of the motors to specific given power.
     *
     * @param power A double between -1.0 and 1.0 which represents the speed that the motors will
     *              be set to.
     */
    public void setPowers(double power){
        if (!robotIsInitialized) throw new Error("Robot not initialized");
        leftFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
        rightFrontDrive.setPower(power);
        rightBackDrive.setPower(power);
        if (power != 0){
//            soundPlayer.startPlaying(hwMap.appContext, pacmanChomp);
        }else{
            soundPlayer.stopPlayingAll();
        }
    }

    /**
     * This method sets the drive behavior of what one of the drive motors of the robot are to do
     * when its power is set to zero.
     *
     * @param behavior A ZeroPowerBehavior enumeration (Under DcMotor) which will be applied to all
     *                 of the drive motors of the robot.
     */
    public void setDriveZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior){
        leftFrontDrive.setZeroPowerBehavior(behavior);
        rightFrontDrive.setZeroPowerBehavior(behavior);
        leftBackDrive.setZeroPowerBehavior(behavior);
        rightBackDrive.setZeroPowerBehavior(behavior);
    }

}
