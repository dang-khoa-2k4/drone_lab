import RPi.GPIO as GPIO
import time

class ButtonReader:
    def __init__(self, pin, bounce_time=0.1):
        self.button_pin = pin
        self.bounce_time = bounce_time
        self.setup_gpio()
        
    def setup_gpio(self):
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)  # Use BCM numbering
        GPIO.setup(self.button_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)  # Set pin as input with pull-down resistor

    def read_button(self):
        return GPIO.input(self.button_pin)

    def start(self):
        try:
            while True:
                button_state = self.read_button()
                
                if button_state == GPIO.LOW:
                    print("Button Pressed")
                else:
                    print("Button Released")
                
                time.sleep(self.bounce_time)  # Small delay to prevent bouncing issues
        except KeyboardInterrupt:
            print("Exiting program")
        finally:
            self.cleanup()

    def cleanup(self):
        GPIO.cleanup()  # Clean up GPIO on exit

class LD2410B:
    def __init__(self, pin, bounce_time=0.1):
        self.button_pin = pin
        self.bounce_time = bounce_time
        self.setup_gpio()
        
    def setup_gpio(self):
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)  # Use BCM numbering
        GPIO.setup(self.button_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)  # Set pin as input with pull-down resistor

    def read_LD2410B(self):
        return GPIO.input(self.button_pin)

    def cleanup(self):
        GPIO.cleanup()  # Clean up GPIO on exit


# Usage
if __name__ == "__main__":
    button_1 = ButtonReader(pin=5)
    button_2 = ButtonReader(pin=6)
    button_3 = ButtonReader(pin=13)
    try:
        while True:
            button_state_1 = button_1.read_button()
            button_state_2 = button_2.read_button()
            button_state_3 = button_3.read_button()
            
            print(f"data: {button_state_1} - {button_state_2} - {button_state_3}")
            
            time.sleep(button_1.bounce_time)  # Small delay to prevent bouncing issues
    except KeyboardInterrupt:
        print("Exiting program")
    finally:
        button_1.cleanup()
