%% MATLAB SOFTUART CHANNEL BIT-ERROR RATE CHARACTERIZER 
%  By Kevin Harper (04/27/2024)
%  ELEC3320, Random Signals Analysis, Prof. Shayok Mukhopadhyay

%% DO NOT CHANGE THESE
DELIM_CHAR = '/';
HW_UART_BAUD = 115200;

%% EDIT AS NEEDED (SEE DEVMGR IN WINDOWS, UNPLUG AND REPLUG DEVICE). FOR LINUX, YOU'RE SCREWED
COM_CHANN    = "COM10";
s = serialport(COM_CHANN, HW_UART_BAUD); % Init HW serial port

%% TEST PARAMS. ADJUST PER TEST.
duration             = 10000;      % Test duration in transactions
data_len_bytes       = 8;          % Length of transaction data. Must match with actual length of data below.
count_mode           = 0;          % Not yet implemented. Replace predetermined transaction data with running count.
data                 = "A5A5A5A5"; % Actual transaction data bytes. Length should match with data_len_bytes
baud                 = 115200;     % Baud rate for test.
transaction_delay_ms = 0;          % Transaction delay (ms) between each transaction (+ overhead due to light processing and forwarding to MATLAB from master)

%% BUILD UP TEST CONFIGURATION DATA TO BE SENT TO MASTER VIA HW UART.
%  FORMAT: duration/data_len_bytes/count_mode/data/baud/transaction_delay_ms(/CRC)
newTestCfg = formatTestData(duration, data_len_bytes, count_mode, data, baud, transaction_delay_ms, DELIM_CHAR);
disp(newTestCfg);
crc = computeCRC(newTestCfg);
fprintf('CRC16 checksum for "%s" is %s\n', newTestCfg, dec2hex(crc));
newTestCfg_CRC = strjoin([newTestCfg, dec2hex(crc)], DELIM_CHAR);
fprintf('Configuration string is %s\n', newTestCfg_CRC);

%% TRANSMIT TO MASTER
writeline(s, newTestCfg_CRC);

readline(s)

% Parameters
n = 16; % Number of trials (bits per byte)
num_samples = 10000; % Number of samples

%% Here we must choose a data string and send it to the master for configuration

% Generate random number of bit errors per sample
num_bit_errors = randi([0, n], num_samples, 1);

% Calculate the probability p based on the generated data
p_estimate = sum(num_bit_errors) / (n * num_samples);

% Display the estimated value of p
disp(['Estimated p: ', num2str(p_estimate)]);

% Possible number of bit errors
k = 0:n;

%% Above we will be looping for the tests duration, adding num errors to running count as we go, then plotting everything at the end

% Probability Mass Function (PMF) of the binomial distribution using estimated p
pmf = binopdf(k, n, p_estimate);

% Plot the PMF
figure;
h = bar(k, pmf, 'b', 'BarWidth', 1);
xlabel('Number of Bit Errors');
ylabel('Probability');
title('Probability Mass Function (PMF) of Bit Errors');
grid on;
hold on;

% Get position of top right corner of the axis
ax_pos = get(gca, 'Position');
ax_width = ax_pos(3);
ax_height = ax_pos(4);
ax_x = ax_pos(1);
ax_y = ax_pos(2);

% Define position of text box
textbox_width = 0.25;
textbox_height = 0.2;
textbox_x = ax_x + ax_width - textbox_width;
textbox_y = ax_y + ax_height - textbox_height - .02;
textbox_position = [textbox_x, textbox_y, textbox_width, textbox_height];

% Calculate padding length for alignment
length_label = max([numel('Length (Bytes) '), numel('Data '), numel('Num Samples '), numel('Baud Rate '), numel('Time Elapsed (s)  '), numel('Estimated p ')]);

% Add key information as a text box
key_str = {['Length (Bytes)', repmat(' ', 1, length_label - numel('Length (Bytes) ')), ': ', num2str(n)]; ...
           ['Data', repmat(' ', 1, length_label - numel('Data ')), ': ', num2str(n)]; ...
           ['Baud Rate', repmat(' ', 1, length_label - numel('Baud Rate ')), ': ', num2str(n)]; ...
           ['Num Samples', repmat(' ', 1, length_label - numel('Num Samples ')), ': ', num2str(num_samples)]; ...
           ['Time Elapsed (s)', repmat(' ', 1, length_label - numel('Time Elapsed (s) ')), ': ', num2str(num_samples)]; ...
           ['Estimated p', repmat(' ', 1, length_label - numel('Estimated p ')), ': ', num2str(p_estimate)]};
textbox_handle = annotation('textbox', textbox_position, 'String', key_str, 'FitBoxToText', 'on');
textbox_handle.FontSize = 10;
textbox_handle.FontName = 'Courier New';  % Use monospaced font
textbox_handle.BackgroundColor = 'white'; % Set background color to white

hold off;

function formatted_string = formatTestData(duration, data_len_bytes, count_mode, data, baud, transaction_delay_ms, delimChar)
    % Convert numerical values to strings
    duration_str = num2str(duration);
    data_len_bytes_str = num2str(data_len_bytes);
    count_mode_str = num2str(count_mode);
    %data_str = (num2str(data));
    baud_str = num2str(baud);
    transaction_delay_ms_str = num2str(transaction_delay_ms);

    % Construct the formatted string with delimiters
    formatted_data = [duration_str delimChar data_len_bytes_str delimChar count_mode_str delimChar data delimChar baud_str delimChar transaction_delay_ms_str];
    
    % Create the actual string
    formatted_string = formatted_data.join('');
end

function crc = computeCRC(data)
    crc = uint16(hex2dec('FFFF')); % Initial value
    ascii_data = convertStringsToChars(data);

    % Update CRC with each byte
    for i = 1:numel(ascii_data)
        crc = crc16_update(crc, ascii_data(i));
    end
end

function crc = crc16_update(crc, a)
    crc = bitxor(crc, uint16(a)); % XOR the current CRC with the byte
    for i = 1:8
        if bitand(crc, 1)
            crc = bitxor(bitshift(crc, -1), hex2dec('A001')); % If the LSB is 1, perform XOR with polynomial
        else
            crc = bitshift(crc, -1); % Shift right by 1
        end
    end
end
