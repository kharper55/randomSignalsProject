%%change all integer formatting to use hex base
data = '6500/8/0/A5A5A5A5/58600/0'; %% 
%% 6500/8/0/A5A5A5A5/58600/0/8C30
%data = '10000/4/0/5555/115200/10/4FD8'; %%    Example data + correct checksum per CRC16 CITT via 'A001' opolynomial
%data = '10000/4/0/5555/115200/10';
crc = computeCRC(data);
disp(['CRC16 checksum for "', data, '" is ', dec2hex(crc)]);

% Create a new string with CRC appended
newData = strcat(data, '/', dec2hex(crc));
% Display the new string
fprintf('New data with CRC: %s\n', newData);


function crc = computeCRC(data)
    crc = uint16(hex2dec('FFFF')); % Initial value

    % Convert data to ASCII bytes
    ascii_data = uint8(data);

    % Update CRC with each byte
    for i = 1:length(ascii_data)
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

