%% Run the Q3.4 observer-based cases and export the report figures
%   Q2.5 (run_observer = 0) figures come from plots_*_case.m; run main_SFC directly for those.
% main_SFC clears everything except batch_case, so the loop state lives there.
batch_case = [1 1 1];
while batch_case(3) <= 2
    main_SFC
    close all
    cases = [1 1; 0 1; 0 0];
    batch_case = [cases(batch_case(3) + 1, :) batch_case(3) + 1];
end
clear batch_case cases
