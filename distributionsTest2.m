% Step 1: Prepare Data
byte_size = 8; % bits
transaction_length_bytes = 4;
transaction_length_bits = transaction_length_bytes * byte_size;
num_trials = 1000;
data = sprintf("%x%x%x%x",5,5,5,5);

config_data = sprintf("%d%s", num_trials, data); %% Send this to Arduino Master via USB UART to intialize and automatically start a test

%% NEED TO GET THE FOLLOWING DATA FROM THE SOFTUART TRANSACTIONS\
%% TRANSACTION NUM, TOTAL NUM BIT ERRORS IN THE TRANSACTION (0 -> TRANSACTION_LEN * 8) -- ESTIMATE P
%% DIV BY TOTAL NUM TRANSACTIONS AND AVERAGE THE FREQUENCY OF NUM BIT ERRORS 
%% This will occur in a running loop

%% if fail, PROBLEMATIC BIT LOCS IN THE TRANSACTION ((BIT 0) -> TRANSACTION_LEN * 8 (HIGHEST ORDER BIT))-- ESTIMATE P
%% DIV BY TOTAL NUM BAD TRANSACTIONS, AVERAGE VIA ALL OF THE SAMPLES CLASSIFIED AS SUCH

% Example data generation for demonstration purposes
num_errors_per_transaction = (0:32); % Number of bit errors

% Initialize frequency vector with zeros for each trial
frequencies = zeros(num_trials, length(num_errors_per_transaction));

% Generate random frequency values for each trial
for i = 1:num_trials
    % Generate random number of errors for each trial
    if i > num_trials / 2
        num_errors = randi([0, transaction_length_bits], 1);
    else
        num_errors = randi([0, transaction_length_bits/2], 1);
    end
    % Calculate the frequency of errors
    frequency = zeros(1, length(num_errors_per_transaction));
    frequency(num_errors + 1) = 1; % Increment frequency at the index corresponding to the number of errors
    frequencies(i, :) = frequency;
end

% Calculate the average frequency across all trials
average_frequency = mean(frequencies, 1);

% Step 2: Calculate Empirical PMF
total_trials = sum(average_frequency); % Total number of trials
empirical_pmf = average_frequency / total_trials; % Empirical PMF

% Step 3: Fit Binomial Distribution
% Here, we'll use the method of moments to estimate p
p_est = sum(num_errors_per_transaction .* average_frequency) / (transaction_length_bits * total_trials); % Method of moments estimator for p

% Step 4: Visualize the Fit
% Plot the empirical PMF and the PMF of the fitted binomial distribution for comparison
figure;

% Plot empirical PMF
stem(num_errors_per_transaction, empirical_pmf, 'filled', 'MarkerSize', 3); % Reduce marker size
xlabel('Number of Bit Errors');
ylabel('Probability Mass Function (PMF)');
title('Empirical PMF of Number of Bit Errors');

% Plot PMF of fitted binomial distribution
binomial_pmf = binopdf(num_errors_per_transaction, transaction_length_bits, p_est); % Assuming 32-bit transactions
hold on;
stem(num_errors_per_transaction, binomial_pmf, 'r', 'LineWidth', 1); % Plot binomial PMF
legend('Empirical PMF', 'Fitted Binomial PMF');
hold off;
