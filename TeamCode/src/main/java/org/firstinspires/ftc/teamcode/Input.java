package org.firstinspires.ftc.teamcode;

import android.os.Debug;

import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * This class manages input abstractions.
 */
public class Input extends Gamepad {

    //The instance of the gamepad that this instance controls.
    public Gamepad gamepad;

    //All the buttons and inputs on the gamepad
    public Button a, b, back, dpad_down, dpad_left, dpad_right, dpad_up, guide, left_bumper, left_stick_button, right_bumper, right_stick_button, start, x, y;
    public FloatButton left_stick_x, left_stick_y, left_trigger, right_stick_x, right_stick_y, right_trigger;

    //An array of all the buttons on the gamepad for looping
    private Button[] buttons;

    /**
     * This is the main constructor, which creates an instance of this class based on the given
     * gamepad instance.
     *
     * @param gamepad The instance of the gamepad that this instance controls.
     */
    public Input(Gamepad gamepad){
        this.gamepad = gamepad;

        //All of the inputs on the robot
        a = new Button(this, Buttons.A);
        b = new Button(this, Buttons.B);
        back = new Button(this, Buttons.BACK);
        dpad_down = new Button(this, Buttons.DPAD_DOWN);
        dpad_left = new Button(this, Buttons.DPAD_LEFT);
        dpad_right = new Button(this, Buttons.DPAD_RIGHT);
        dpad_up = new Button(this, Buttons.DPAD_UP);
        guide = new Button(this, Buttons.GUIDE);
        left_bumper = new Button(this, Buttons.LEFT_BUMPER);
        left_stick_button = new Button(this, Buttons.LEFT_STICK_BUTTON);
        right_bumper = new Button(this, Buttons.RIGHT_BUMPER);
        right_stick_button = new Button(this, Buttons.RIGHT_STICK_BUTTON);
        start = new Button(this, Buttons.START);
        x = new Button(this, Buttons.X);
        y = new Button(this, Buttons.Y);
        left_stick_x = new FloatButton(this, FloatButtons.LEFT_STICK_X);
        left_stick_y = new FloatButton(this, FloatButtons.LEFT_STICK_Y);
        left_trigger = new FloatButton(this, FloatButtons.LEFT_TRIGGER);
        right_stick_x = new FloatButton(this, FloatButtons.RIGHT_STICK_X);
        right_stick_y = new FloatButton(this, FloatButtons.RIGHT_STICK_Y);
        right_trigger = new FloatButton(this, FloatButtons.RIGHT_TRIGGER);

        buttons = new Button[]{a, b, back, dpad_down, dpad_left, dpad_right, dpad_up, guide, left_bumper, left_stick_button, right_bumper, right_stick_button, start, x, y, left_stick_x, left_stick_y, left_trigger, right_stick_x, right_stick_y, right_trigger};
    }

    /**
     * This method updates all of the button values
     */
    public void updateAll(){
        for (Button button : buttons){
            button.update();
        }
    }

}
