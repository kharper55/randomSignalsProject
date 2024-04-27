% Parameters
num_transactions = 10000; % Number of transactions
byte_length = 8; % Byte length for each transaction
error_rate = 0.1; % Error rate (probability of error for each bit)

% Initialize arrays to store bit error rates
bit_error_rates = zeros(1, num_transactions);

% Loop through each transaction
for i = 1:num_transactions
    % Generate random data for the transaction
    transmitted_data = randi([0, 1], 1, byte_length);
    
    % Introduce random errors based on error rate
    received_data = transmitted_data;
    for j = 1:length(received_data)
        if rand < error_rate
            received_data(j) = ~received_data(j); % Flip the bit
        end
    end
    
    % Calculate bit error rate for the transaction
    bit_error_rate = nnz(transmitted_data ~= received_data);
    bit_error_rates(i) = bit_error_rate;
end

% Calculate the probability mass function (PMF) of bit error rates
max_bit_errors = max(bit_error_rates);
pmf = zeros(1, byte_length + 1); % Adjusted to byte_length
for i = 0:byte_length % Adjusted to byte_length
    pmf(i + 1) = sum(bit_error_rates == i) / num_transactions;
end

% Plot the PMF
figure;
bar(0:byte_length, pmf, 0.5); % Adjusted width of the bars
xlabel('Number of Bit Errors');
ylabel('Probability');
title('Probability Mass Function (PMF) of Bit Error Rates');
xlim([-0.5, byte_length + 0.5]); % Adjusted to show all indices on x-axis

% Plot fitted binomial distribution
hold on;
plot(0:max_bit_errors, pdf(binomial_dist, 0:max_bit_errors), 'r-', 'LineWidth', 2);
legend('Data', 'Fitted Binomial Distribution');
grid on;
hold off;
