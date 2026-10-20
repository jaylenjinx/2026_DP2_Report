%% Run every DP2 case and export all report figures
%   Q2.5: SFC only (linear, nonlinear)    Q3.4: SFC + observer (linear, nonlinear)
% main_SFC clears everything except batch_case, so the loop state lives there.
batch_case = [1 0 1];
while batch_case(3) <= 4
    main_SFC
    close all
    cases = [1 0; 0 0; 1 1; 0 1; 0 0];
    batch_case = [cases(batch_case(3) + 1, :) batch_case(3) + 1];
end
clear batch_case cases
