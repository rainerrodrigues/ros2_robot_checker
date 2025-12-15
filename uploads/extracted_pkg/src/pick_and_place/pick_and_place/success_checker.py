
import time

def main():
    time.sleep(5)
    with open('/tmp/success.txt', 'w') as f:
        f.write('SUCCESS')

