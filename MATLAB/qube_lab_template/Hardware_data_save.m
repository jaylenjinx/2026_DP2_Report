%% Incremental Experimental Data Saving
%
% 41277 Control Design
% University of Technology Sydney, Australia
%
% A/Prof Ricardo Aguilera

% This script will run after stopping the Simulink Model.  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% save data.
if exist('save_data', 'var')
if save_data
    file_name = "data_test";
    count = 1;
    
    check_file = true;
    while(check_file)
        if count<10
            file_full = file_name + "_0" + num2str(count) + ".mat";    
        else
            file_full = file_name + "_" + num2str(count) + ".mat";   
        end
        if  (exist(file_full,'file')>0)
            count = count + 1;     
        else 
            check_file = false;
        end
    end
    save(file_full);

end
end