from sensor_msgs.msg import Joy


class LogitechF710:
    def __init__(self):
        self._joy_state = {}
        self._prev_joy_state = {}

    # Getter for _joy_state
    @property
    def joy_state(self):
        return self._joy_state

    # Setter for _joy_state
    @joy_state.setter
    def joy_state(self, value):
        if isinstance(value, dict):
            self._prev_joy_state = self._joy_state.copy()
            self._joy_state = value
        else:
            raise ValueError("joy_state must be a dictionary")

    def map(self, joy_msg: Joy):
        """
        Map the joystick message to a control button.
        :param joy_msg: The joystick message.
        :return: joy button value Dict
        """
        return {
            "button_a": joy_msg.buttons[1],
            "button_b": joy_msg.buttons[2],
            "button_x": joy_msg.buttons[0],
            "button_y": joy_msg.buttons[3],
            "button_left": joy_msg.buttons[4],
            "button_right": joy_msg.buttons[5],
            "button_left_back": joy_msg.buttons[6],
            "button_right_back": joy_msg.buttons[7],
            "button_back": joy_msg.buttons[8],
            "button_start": joy_msg.buttons[9],
            "d_pad_x": joy_msg.axes[0],
            "d_pad_y": joy_msg.axes[1],
            "analog_left_x": joy_msg.axes[4],
            "analog_left_y": joy_msg.axes[5],
            "analog_right_x": joy_msg.axes[2],
            "analog_right_y": joy_msg.axes[3],
        }

    def is_press(self, button_key):
        cur_value = self._joy_state.get(button_key, 0)
        prev_value = self._prev_joy_state.get(button_key, 0)
        if cur_value == 0 and prev_value == 1:
            return True
        else:
            return False

    def value(self, button_key):
        """
        Get the value of a button.
        :param button_key: The key of the button.
        :return: The value of the button.
        """
        return self._joy_state.get(button_key, 0)
