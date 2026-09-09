%% save PID gains after tuning
% god bless chatGPT
% Get the tuned PID gains from Simulink
blockX = 'PID/PID x';  
blockY = 'PID/PID y';
blockZ = 'PID/PID z';
blocku = 'PID/PID u';  
blockv = 'PID/PID v';
blockw = 'PID/PID w';
blockpitch = 'PID/PID pitch';  
blockroll = 'PID/PID roll';
blockyaw = 'PID/PID yaw';
blockP = 'PID/PID p';
blockQ = 'PID/PID q';
blockR = 'PID/PID r';

% Define all PID block paths
pid_blocks = struct(...
    'x', 'PID/PID x', ...
    'y', 'PID/PID y', ...
    'z', 'PID/PID z', ...
    'u', 'PID/PID u', ...
    'v', 'PID/PID v', ...
    'w', 'PID/PID w', ...
    'pitch', 'PID/PID pitch', ...
    'roll', 'PID/PID roll', ...
    'yaw', 'PID/PID yaw', ...
    'p', 'PID/PID p', ...
    'q', 'PID/PID q', ...
    'r', 'PID/PID r' ...
);

% Initialize a structure to store the PID gains
PID_Gains = struct();

% Loop through each PID block and extract the gains
fields = fieldnames(pid_blocks);
for i = 1:length(fields)
    block_name = fields{i};
    block_path = pid_blocks.(block_name);
    
    % Extract PID gains using get_param
    Kp = str2double(get_param(block_path, 'P'));
    Ki = str2double(get_param(block_path, 'I'));
    Kd = str2double(get_param(block_path, 'D'));
    N  = str2double(get_param(block_path, 'N')); 

    % Store gains in the structure
    PID_Gains.(block_name) = struct('Kp', Kp, 'Ki', Ki, 'Kd', Kd, "N",N);
    
    % Display extracted gains
    fprintf('Block: %s\nKp: %.6f, Ki: %.6f, Kd: %.6f, N: %.6f\n\n', block_name, Kp, Ki, Kd, N);

end

% Save the PID gains for later use
save('PID_Gains_TVT_paper_with_hold.mat', 'PID_Gains');
disp('PID gains saved to PID_Gains.mat');