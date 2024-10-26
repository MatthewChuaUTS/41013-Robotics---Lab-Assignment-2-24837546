device = serialport("COM10",9600); % initiate connection to ESP (run this once)
flush(device); 

while true
    % make the below run throughout your program
    flush(device);    % clears the serial buffer to read the latest msg from ESP
    read(device,1,"char") % msg received from the ESP (0 or 1)
    pause(0.1);
end