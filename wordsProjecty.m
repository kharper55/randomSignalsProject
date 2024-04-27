% Parameters
n = 8; % Number of trials (bits per byte)
num_samples = 10000; % Number of samples

% Generate random number of bit errors per sample
num_bit_errors = randi([0, n], num_samples, 1);

% Calculate the probability p based on the generated data
p_estimate = sum(num_bit_errors) / (n * num_samples);

% Display the estimated value of p
disp(['Estimated p: ', num2str(p_estimate)]);

% Possible number of bit errors
k = 0:n;

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
textbox_x = ax_x + ax_width - textbox_width; % Align right edge of textbox with right edge of inner graph window
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
textbox_handle = annotation('textbox', 'Position', textbox_position, 'String', key_str, 'FitBoxToText', 'on');
textbox_handle.FontSize = 10;
textbox_handle.FontName = 'Courier New';  % Use monospaced font
textbox_handle.BackgroundColor = 'white'; % Set background color to white

hold off;
