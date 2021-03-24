package org.firstinspires.ftc.teamcode;

/**
 * This class simply allows multithreading to shutdown TensorFlow object detection asynchronously.
 *
 * @author Arkin Solomon
 */
public class ShutdownTFOD extends Thread {

    protected HardwareSPQR robot;

    public ShutdownTFOD (HardwareSPQR robot){ this.robot = robot; }

    @Override
    public void run() {
        this.robot.tfod.deactivate();
        this.robot.tfod.shutdown();
    }
}
