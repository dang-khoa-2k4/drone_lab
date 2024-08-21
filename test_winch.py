import subprocess

class YourClass:
    def __init__(self):
        self.modeWinch = "manual"  # Example: Set the initial mode to "manual"
        self.stepper = YourStepperClass()  # Replace with your stepper motor class instance

    def process_payload(self, payload):
        if payload["header"] == "runWinch":
            if self.modeWinch == "manual":
                self.stepper.move_clockwise(int(payload['data']))
                print(f"run stepper: {payload['data']}")

                # Run the external Python script
                try:
                    subprocess.run(["python3", "control.py"])
                    print("Executed other_code.py successfully")
                except Exception as e:
                    print(f"Failed to run other_code.py: {e}")

class YourStepperClass:
    def move_clockwise(self, steps):
        # Your implementation to move the stepper motor clockwise
        print(f"Moving stepper motor clockwise by {steps} steps")

# Example usage
your_class_instance = YourClass()
payload = {"header": "runWinch", "data": "100"}  # Example payload
your_class_instance.process_payload(payload)
