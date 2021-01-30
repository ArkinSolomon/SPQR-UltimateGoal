package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * This class manages input abstractions.
 */
public class Input extends Gamepad {

    //The instance of the gamepad that this instance controls.
    public Gamepad gamepad;

    //All of the inputs on the robot
    public Button a = new Button(this, Buttons.A);
    public Button b = new Button(this, Buttons.B);
    public Button back = new Button(this, Buttons.BACK);
    public Button dpad_down = new Button(this, Buttons.DPAD_DOWN);
    public Button dpad_left = new Button(this, Buttons.DPAD_LEFT);
    public Button dpad_right = new Button(this, Buttons.DPAD_RIGHT);
    public Button dpad_up = new Button(this, Buttons.DPAD_UP);
    public Button guide = new Button(this, Buttons.GUIDE);
    public Button left_bumper = new Button(this, Buttons.LEFT_BUMPER);
    public Button left_stick_button = new Button(this, Buttons.LEFT_STICK_BUTTON);
    public Button right_bumper = new Button(this, Buttons.RIGHT_BUMPER);
    public Button right_stick_button = new Button(this, Buttons.RIGHT_STICK_BUTTON);
    public Button start = new Button(this, Buttons.START);
    public Button x = new Button(this, Buttons.X);
    public Button y = new Button(this, Buttons.Y);
    public FloatButton left_stick_x = new FloatButton(this, FloatButtons.LEFT_STICK_X);
    public FloatButton left_stick_y = new FloatButton(this, FloatButtons.LEFT_STICK_Y);
    public FloatButton left_trigger = new FloatButton(this, FloatButtons.LEFT_TRIGGER);
    public FloatButton right_stick_x = new FloatButton(this, FloatButtons.RIGHT_STICK_X);
    public FloatButton right_stick_y = new FloatButton(this, FloatButtons.RIGHT_STICK_Y);
    public FloatButton right_trigger = new FloatButton(this, FloatButtons.RIGHT_TRIGGER);

    //An array of all the buttons on the robot for looping
    public Button[] buttons = {a, b, back, dpad_down, dpad_left, dpad_right, dpad_up, guide, left_bumper, left_stick_button, right_bumper, right_stick_button, start, x, y, left_stick_x, left_stick_y, left_trigger, right_stick_x, right_stick_y, right_trigger};

    /**
     * This is the main constructor, which creates an instance of this class based on the given
     * gamepad instance.
     *
     * @param gamepad The instance of the gamepad that this instance controls.
     */
    public Input(Gamepad gamepad){
        gamepad = this.gamepad;
    }

    /**
     * This method updates all of the button values
     */
    public void update() throws Exception{
        for (Button button : buttons){
            button.update();
        }
    }

}
