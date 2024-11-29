# -*- coding: utf-8 -*-
'''
 ------------------------------------------------------------------
 @File Name:        ps_controller
 @Created:          2024 2024/10/30 12:06
 @Software:         PyCharm
 
 @Author:           Jiayu ZENG
 @Email:            jiayuzeng@asagi.waseda.jp
 
 @Description:      

 ------------------------------------------------------------------
'''

import pygame

class PS2Controller:
    def __init__(self):
        pygame.init()
        pygame.joystick.init()
        self.clock = pygame.time.Clock()
        self.joysticks = []
        self.button_states = []  # To store button states
        self.hat_states = []  # To store hat (D-pad) states
        self._initialize_joysticks()

    def _initialize_joysticks(self):
        joystick_count = pygame.joystick.get_count()
        for i in range(joystick_count):
            joystick = pygame.joystick.Joystick(i)
            joystick.init()
            self.joysticks.append(joystick)
            self.button_states.append([False] * joystick.get_numbuttons())  # Initialize button states
            self.hat_states.append([0, 0])  # Initialize hat states (assuming one hat per joystick)

    def get_joystick_info(self):
        joystick_info = []
        for joystick in self.joysticks:
            info = {
                'name': joystick.get_name(),
                'axes': [joystick.get_axis(i) for i in range(joystick.get_numaxes())],
                'buttons': [joystick.get_button(i) for i in range(joystick.get_numbuttons())],
                'hats': [joystick.get_hat(i) for i in range(joystick.get_numhats())],
            }
            joystick_info.append(info)
        return joystick_info

    def get_left_stick(self, joystick_index=0):
        if joystick_index < len(self.joysticks):
            joystick = self.joysticks[joystick_index]
            return joystick.get_axis(0), joystick.get_axis(1)  # Left stick usually uses axes 0 and 1
        return None, None

    def get_right_stick(self, joystick_index=0):
        if joystick_index < len(self.joysticks):
            joystick = self.joysticks[joystick_index]
            return joystick.get_axis(2), joystick.get_axis(3)  # Right stick usually uses axes 2 and 3
        return None, None

    def get_dpad(self, joystick_index=0):
        if joystick_index < len(self.hat_states):
            return self.hat_states[joystick_index]
        return None, None

    def update_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False
            elif event.type == pygame.JOYBUTTONDOWN:
                joystick_index = event.joy
                button_index = event.button
                self.button_states[joystick_index][button_index] = True
                print(f"Joystick {joystick_index} button {button_index} pressed.")
            elif event.type == pygame.JOYBUTTONUP:
                joystick_index = event.joy
                button_index = event.button
                self.button_states[joystick_index][button_index] = False
                print(f"Joystick {joystick_index} button {button_index} released.")
            elif event.type == pygame.JOYHATMOTION:
                joystick_index = event.joy
                hat_index = event.hat
                self.hat_states[joystick_index] = event.value
                print(f"Joystick {joystick_index} hat {hat_index} moved to {event.value}.")
        return True

    def get_button_states(self, joystick_index=0):
        if joystick_index < len(self.button_states):
            return self.button_states[joystick_index]
        return []

    def close(self):
        pygame.quit()

    def run_non_blocking(self):
        self.update_events()
        info = self.get_joystick_info()
        for i, joystick in enumerate(info):
            print(f"Joystick {i}: {joystick}")
            left_stick = self.get_left_stick(i)
            right_stick = self.get_right_stick(i)
            dpad = self.get_dpad(i)
            button_states = self.get_button_states(i)
            print(f"Left Stick: {left_stick}")
            print(f"Right Stick: {right_stick}")
            print(f"D-pad: {dpad}")
            print(f"Button States: {button_states}")

if __name__ == "__main__":
    controller = PS2Controller()
    try:
        while True:
            controller.run_non_blocking()
            controller.clock.tick(20)
    except KeyboardInterrupt:
        controller.close()
