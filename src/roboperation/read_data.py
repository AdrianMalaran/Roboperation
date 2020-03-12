##############
## Script listens to serial port and writes contents into a file
##############
## requires pySerial to be installed
import serial  # sudo pip install pyserial should work

serial_port = '/dev/ttyACM0';
baud_rate = 9600; #In arduino, Serial.begin(baud_rate)
write_to_file_path = "/home/robohub/robohub/panda/Roboperation/src/roboperation/output.txt";



ser = serial.Serial(serial_port, baud_rate)
while True:
    output_file = open(write_to_file_path, "a");
    line = ser.readline()
    # line = line.decode("utf-8") #ser.readline returns a binary, convert to string
    print(line);
    output_file.write(line)
    output_file.close()

    # Force Feedback Communication Script
    # force_f = open("/home/robohub/robohub/panda/Roboperation/src/roboperation/force_output.txt", 'r')
    # content = force_f.read()
    # force_f.close()
    # # print(content)
    #
    # new_force_f = open("/home/robohub/robohub/panda/Roboperation/src/roboperation/force_output.txt", 'w')
    # new_force_f.write("")
    # new_force_f.close()
    # if content.split(",")[0].strip() == "HIT":
    #     ser.write(1)
