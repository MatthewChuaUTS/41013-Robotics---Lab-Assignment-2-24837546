device = serialport("COM5",9600); % initiate connection to ESP (run this once)

% make the below run throughout your program
while 1
    flush(device);    % clears the serial buffer to read the latest msg from ESP
    % out = readDigitalPin(device,'D13');
    out = read(device,1,"char"); % msg received from the ESP (0 or 1)
    disp(out);
    % pause(0.01);
end

% esp32 = arduino('COM5','ESP32-WROOM-DevKitV1'); % initiate connection to ESP (run this once)
% 
% % make the below run throughout your program
% while 1
%     flush(esp32);    % clears the serial buffer to read the latest msg from ESP
%     % out = readDigitalPin(device,'D13');
%     out = read(esp32,1,"char"); % msg received from the ESP (0 or 1)
%     disp(out);
%     pause(0.25);
% end
% 
% 
% flush(esp32); 
% hardwareEstopValue = read(esp32,1,"char");


