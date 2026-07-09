import sys
import time

def main():
    print("=" * 60)
    print("      R17-D UHF RFID Reader - Auto Reader Console")
    print("=" * 60)
    print("[*] Status: Reader is ready.")
    print("[*] Instruction: Keep this console window focused/active.")
    print("[*] Wave an RFID tag in front of the reader to scan...\n")

    try:
        while True:
            # Since the reader simulates keyboard typing and presses Enter automatically,
            # input() will capture the full ID instantly without any manual intervention.
            tag_id = input().strip()
            if tag_id:
                timestamp = time.strftime('%Y-%m-%d %H:%M:%S')
                print(f"[{timestamp}] Tag Detected -> EPC: {tag_id} (Length: {len(tag_id)} chars)")
                # Flush output so it prints immediately in all terminals
                sys.stdout.flush()
    except KeyboardInterrupt:
        print("\n[*] Exiting auto reader script. Goodbye!")

if __name__ == "__main__":
    main()
