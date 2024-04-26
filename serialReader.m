s = serialport("COM8",115200)
count = 0;

% Accept data from serial port. Serial port communicates the current byte
% number and whether or not an error occurred. Expand to include more
% information for a mroe robust characterization.
while (count < 10) 
    string = sprintf("HELP%d", count);
    writeline(s, string);
    readline(s)
    count=count+1;
end

delete(s)
%% NOTE:: IF MATLAB CANT ACCESS COM PORT, JUST PASTE THIS LINE IN THE COMMAND WINDOW