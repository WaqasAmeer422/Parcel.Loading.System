import time
import lgpio

# Configuration
MOTOR_PIN = 23  # GPIO 23 (RPWM on the IBT-2)
GPIOCHIP = 0    # Set to 4 if you are running on a Raspberry Pi 5

# Initialize the GPIO chip
try:
    h = lgpio.gpiochip_open(GPIOCHIP)
    lgpio.gpio_claim_output(h, MOTOR_PIN)
    print(f"[Motor Test] GPIO {MOTOR_PIN} claimed successfully on chip {GPIOCHIP}.")
except Exception as e:
    print(f"[ERROR] Could not open GPIO chip: {e}")
    exit(1)

try:
    print("\n============================================")
    print("          CONVEYOR MOTOR TEST MODE          ")
    print("============================================\n")

    # Step 1: Full Speed Test
    print("[1/3] Running motor at FULL SPEED for 3 seconds...")
    lgpio.gpio_write(h, MOTOR_PIN, 1)  # Write HIGH (Full speed)
    time.sleep(10)

    # Step 2: Stop Test
    print("[2/3] Stopping motor for 2 seconds...")
    lgpio.gpio_write(h, MOTOR_PIN, 0)  # Write LOW (Stop)
    time.sleep(2)

    # Step 3: Speed Control (PWM) Test
    print("[3/3] Running motor at 40% speed using PWM for 3 seconds...")
    lgpio.tx_pwm(h, MOTOR_PIN, 1000, 40)  # 1000Hz frequency, 40% speed
    time.sleep(10)

    print("[3/3] Increasing speed to 80% using PWM for 3 seconds...")
    lgpio.tx_pwm(h, MOTOR_PIN, 1000, 80)  # 1000Hz frequency, 80% speed
    time.sleep(10)

    print("[Motor Test] Stopping motor...")
    lgpio.tx_pwm(h, MOTOR_PIN, 0, 0)  # Turn off PWM
    lgpio.gpio_write(h, MOTOR_PIN, 0)  # Extra safety stop

    print("\n============================================")
    print("           TEST COMPLETE SUCCESS            ")
    print("============================================")

finally:
    # Release GPIO resource
    lgpio.gpiochip_close(h)
    print("[Motor Test] GPIO chip connection closed.")
