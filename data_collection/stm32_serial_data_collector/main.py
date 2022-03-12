import serial
import datetime
import matplotlib.pyplot as plt

NUM_SAMPLES = 2000

compl_data = list()

def generate_and_write_csv():
    print(compl_data)
    result = ""
    count = 0
    for collection in compl_data:
        count += 1
        row0 = f"{count}"
        row1 = f"{count}"
        row2 = f"{count}"
        row3 = f"{count}"
        for r0 in collection[0]:
            row0 += f", {r0}"
        for r1 in collection[1]:
            row1 += f", {r1}"
        for r2 in collection[2]:
            row2 += f", {r2}"
        for r3 in collection[3]:
            row3 += f", {r3}"
        result += f"{row0}\n{row1}\n{row2}\n{row3}\n"
    print(result)
    current_date_time = datetime.datetime.now().strftime("%Y-%m-%d:%H:%M:%S")
    with open(f"{current_date_time}_led_data.csv", "w") as f:
        f.write(result)
    

def main():
    serial_conn = serial.Serial("/dev/ttyACM0", baudrate=115200)
    # naming the x axis
    plt.xlabel("sample")
    # naming the y axis
    plt.ylabel("size")
    
    # giving a title to my graph
    plt.title("LED - result")
    
    plt.ion()
    # function to show the plot
    #plt.show()
    
    cc = 0
    
    dc_data = list()
    led_1_data = list()
    led_2_data = list()
    led_3_data = list()
    
    try:
        while True:
            plt.show()
            
            line_b = serial_conn.readline()
            line = "".join([chr(c) for c in line_b])
            
            if cc % 100 == 0:
                print(line)
            cc += 1
            
            led = None
            num_str = None
            
            try:
                [led, num_str] = line.split(' ')
            except ValueError:
                continue
            finally:
                if led == "LED0:":
                    dc_data.append(int(num_str[0:-1]))
                if led == "LED1:":
                    led_1_data.append(int(num_str[0:-1]))
                elif led == "LED2:":
                    led_2_data.append(int(num_str[0:-1]))
                elif led == "LED3:":
                    led_3_data.append(int(num_str[0:-1]))
            
            # plotting the points
            if len(led_1_data) >= NUM_SAMPLES-100 and len(led_2_data) >= NUM_SAMPLES-100 and len(led_3_data) >= NUM_SAMPLES-100:
                compl_data.append((dc_data.copy(), led_1_data.copy(), led_2_data.copy(), led_3_data.copy()))
                scaled_data_0 = dc_data[::10]
                scaled_data_1 = led_1_data[::10]
                scaled_data_2 = led_2_data[::10]
                scaled_data_3 = led_3_data[::10]
                
                print("plotting data")
                plt.clf()
                plt.plot([i for i in range(len(scaled_data_0))], scaled_data_0, label="dc_data")
                plt.plot([i for i in range(len(scaled_data_1))], scaled_data_1, label="led_1_data")
                plt.plot([i for i in range(len(scaled_data_2))], scaled_data_2, label="led_2_data")
                plt.plot([i for i in range(len(scaled_data_3))], scaled_data_3, label="led_3_data")
                plt.draw()
                plt.pause(1)
            
                dc_data.clear()
                led_1_data.clear()
                led_2_data.clear()
                led_3_data.clear()
        
            #print("done!")
                
            
    except KeyboardInterrupt:
        pass
    
    generate_and_write_csv()
        


if __name__ == "__main__":
    main()
