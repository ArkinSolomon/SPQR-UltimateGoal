package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * This class represents a single button that has a boolean value.
 */
public class Button {

    //The input instance that this button is a part of
    public Input input;

    //The gamepad of the input that this button is a part of (abstraction for input.gamepad)
    public Gamepad gamepad;

    //The button that this instance is controlling
    public Buttons button = Buttons.NONE;

    //If the button is currently done
    public boolean isDown = false;

    //If the button is currently up
    public boolean isUp = true;

    //If this is when we're detecting the button is down for the first time
    public boolean down = false;

    //If this is when we're detecting the button is up for the first time
    public boolean up = true;

    /**
     * The main class constructor, this constructor creates a new instance with the linked input
     * instance.
     *
     * @param input The input that this button is a part of.
     * @param button The button that this button instance is controlling.
     */
    public Button(Input input, Buttons button){
        this.input = input;
        this.gamepad = input.gamepad;
        this.button = button;
    }

    /**
     * This method updates the values of the button.
     */
    public void update() throws Exception{
        boolean isCurrentButtonDown;
        switch (button){
            case A:
                isCurrentButtonDown = gamepad.a;
                break;
            case B:
                isCurrentButtonDown = gamepad.b;
                break;
            case BACK:
                isCurrentButtonDown = gamepad.back;
                break;
            case DPAD_DOWN:
                isCurrentButtonDown = gamepad.dpad_down;
                break;
            case DPAD_LEFT:
                isCurrentButtonDown = gamepad.dpad_left;
                break;
            case DPAD_RIGHT:
                isCurrentButtonDown = gamepad.dpad_right;
                break;
            case DPAD_UP:
                isCurrentButtonDown = gamepad.dpad_up;
                break;
            case GUIDE:
                isCurrentButtonDown = gamepad.guide;
                break;
            case LEFT_BUMPER:
                isCurrentButtonDown = gamepad.left_bumper;
                break;
            case LEFT_STICK_BUTTON:
                isCurrentButtonDown = gamepad.left_stick_button;
                break;
            case RIGHT_BUMPER:
                isCurrentButtonDown = gamepad.right_bumper;
                break;
            case RIGHT_STICK_BUTTON:
                isCurrentButtonDown = gamepad.right_stick_button;
                break;
            case START:
                isCurrentButtonDown = gamepad.start;
                break;
            case X:
                isCurrentButtonDown = gamepad.x;
                break;
            case Y:
                isCurrentButtonDown = gamepad.y;
                break;
            default:
                throw new Exception("Why is the button \"NONE\"?????????");
        }
        updateValues(isCurrentButtonDown);
    }

    /**
     * Update the main values of the class based on if the input is down currently.
     *
     * @param isCurrentButtonDown If the input button is currently down.
     */
    protected void updateValues(boolean isCurrentButtonDown){
        up = isDown && !isCurrentButtonDown;
        down = isUp && isCurrentButtonDown;
        isDown = isCurrentButtonDown;
        isUp = !isDown;
    }
}
