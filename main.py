from capture import adcCapThread
import time
import sys

def main():
    a = adcCapThread(1, "adc")
    a.start()
    counter = 0
    t = time.time()
    tim_min = 0

    with open(str(t).split(".")[0] + ".bin", "wb") as f:
        duration = int(sys.argv[1])

        while True:
            readItem, itemNum, lostPacketFlag = a.getFrame()
            if itemNum > 0:
                counter += 1
                f.write(readItem.tobytes())
                if counter == 600:
                    counter = 0
                    tim_min += 1

            elif itemNum == -1:
                print(readItem)
            elif itemNum == -2:
                time.sleep(0.04)
            if tim_min >= duration:
                a.whileSign = False


if __name__ == "__main__":
    main()
