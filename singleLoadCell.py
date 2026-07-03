  To solve this, we will write the file line-by-line using a series of very short, simple commands. This completely avoids line-wrapping and
  terminal buffer overflow.
  ** use waveshare IO board and Pi compute module 3+.
  ### For data GPIO Bank0 Pin 5, and 6 for clock
  ### Step 1: Copy and paste this block

  Copy this entire block, paste it into your terminal, and press Enter:

    rm -f single_test.py
    echo "import RPi.GPIO as GPIO, time, sys" >> single_test.py
    echo "D, S = 5, 6" >> single_test.py
    echo "GPIO.setmode(GPIO.BCM)" >> single_test.py
    echo "GPIO.setup(S, GPIO.OUT)" >> single_test.py
    echo "GPIO.setup(D, GPIO.IN)" >> single_test.py
    echo "GPIO.output(S, False)" >> single_test.py
    echo "print('Script started successfully!', flush=True)" >> single_test.py
    echo "while True:" >> single_test.py
    echo "    while GPIO.input(D) != 0: pass" >> single_test.py
    echo "    v = 0" >> single_test.py
    echo "    for _ in range(24):" >> single_test.py
    echo "        GPIO.output(S, True); GPIO.output(S, False)" >> single_test.py
    echo "        v = (v << 1) | GPIO.input(D)" >> single_test.py
    echo "    GPIO.output(S, True); GPIO.output(S, False)" >> single_test.py
    echo "    print('Raw Value:', v ^ 0x800000, flush=True)" >> single_test.py
    echo "    time.sleep(0.5)" >> single_test.py

  This will construct a clean, 16-line test script without any indentation or wrapping issues.
  ──────
  ### Step 2: Verify the file contents

  Check that the file was created successfully:

    cat single_test.py

  You should see the clean, uncorrupted python code.
  ──────
  ### Step 3: Run the script

  Run it now with:

    sudo python3 -u single_test.py

  You should see this print immediately:

    Script started successfully!

    
    
    
    
    
    
    
    
    
    
    