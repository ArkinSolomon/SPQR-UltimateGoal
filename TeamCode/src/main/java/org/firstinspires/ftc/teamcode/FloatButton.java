package org.firstinspires.ftc.teamcode;

/**
 * An instance class represents a button which value is stored as a float.
 */
public class FloatButton extends Button {

    //The button that this instance is controlling
    public FloatButtons button = FloatButtons.NONE;

    //The threshold for this button to be considered pressed down
    public double threshold = .15;

    //The value of the input
    public double value = 0;

    /**
     * The main class constructor, this constructor creates a new instance with the linked input
     * instance.
     *
     * @param input The input class instance that this button is a part of.
     * @param button The input that this instance is controlling.
     */
    public FloatButton(Input input, FloatButtons button){
        super(input, Buttons.NONE);
        this.input = input;
        this.gamepad = input.gamepad;
        this.button = button;
    }

    /**
     * This method updates the values of the input.
     */
    @Override
    public void update() throws Exception {
        boolean isCurrentInputDown;
        switch (button){
            case LEFT_STICK_X:
                value = gamepad.left_stick_x;
                isCurrentInputDown = value >= threshold;
                break;
            case LEFT_STICK_Y:
                value = gamepad.left_stick_y;
                isCurrentInputDown = value >= threshold;
                break;
            case LEFT_TRIGGER:
                value = gamepad.left_trigger;
                isCurrentInputDown = value >= threshold;
                break;
            case RIGHT_STICK_X:
                value = gamepad.right_stick_x;
                isCurrentInputDown = value >= threshold;
                break;
            case RIGHT_STICK_Y:
                value = gamepad.right_stick_y;
                isCurrentInputDown = value >= threshold;
                break;
            case RIGHT_TRIGGER:
                value = gamepad.right_trigger;
                isCurrentInputDown = value >= threshold;
                break;
            default:
                throw new Exception("Why is the button \"NONE\"?????????");
        }
        updateValues(isCurrentInputDown);
    }
}
