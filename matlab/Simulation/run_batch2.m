clear
close all
clc

SimData = cell(5000,10);

gate_width = 0.7;
gate_r = 0.025;
drone_r = 0.2;
filename = 'Sim2.mat';

list_speed = [1.0,2.0,3.0,4.0];
list_agility = [3];
list_sensor = [0.01,0.05,0.1];
list_sensor_eul = deg2rad([0.02,0.1,0.4]);
list_actuator = [0.1,0.25,0.4];
list_iterations = 1:100;

i = 1;
for idx_i=1:length(list_iterations)
    for idx_speed=1:length(list_speed)
        for idx_agility=1:length(list_agility)
            for idx_sensor=1:length(list_sensor)
                for idx_actuator=1:length(list_actuator)

                    try
                        initialize_race_realtime

                        %% override
                        max_vel = list_speed(idx_speed);
                        max_theta_X = list_agility(idx_agility)*max_theta_X;
                        max_theta_Y = list_agility(idx_agility)*max_theta_Y;
                        max_v_Z     = list_agility(idx_agility)*max_v_Z;
                        max_rot_Z   = list_agility(idx_agility)*max_rot_Z;

                        max_force = mass*g*tan(max_theta_X);
                        max_moment = 5*list_agility(idx_agility)/2.5;

                        % noise
                        dist = 3; %m
                        std_dev_xyz = list_sensor(idx_sensor)*dist*ones(1,3);
                        std_dev_eul = list_sensor_eul(idx_sensor)*dist*ones(1,3);
                        std_dev_vel = list_sensor(idx_sensor)*dist*ones(1,3);
                        std_dev_F = list_actuator(idx_actuator)*max_force*ones(1,3);
                        std_dev_M = list_actuator(idx_actuator)*max_moment*[0, 0, 1];

                        output = sim('Bebop10_from_exp','TimeOut',100,'ReturnWorkspaceOutputs','on');
                        SimData(i,1) = num2cell(output.sim_idx.Time,1);
                        SimData(i,2) = num2cell(output.sim_idx.Data,1);
                        SimData(i,3) = num2cell(output.sim_state_pos.Time,1);
                        SimData(i,4) = num2cell(output.sim_state_pos.Data,[1,2]);
                        SimData(i,5) = num2cell(output.sim_gates.Data(:,:,1),[1,2]);
                        
                       
                    end
                    
                        i = i+1
%                     %% check collision
%                     % for every line segment check distance to all circles
%                     drone_pos = output.sim_state_pos.Data(20:25,1:2);
%                     gates = output.sim_gates.Data(:,:,1);
%                     drone_pos = drone_pos .* ones(1,1,size(gates,1));
%                     gates_pos = gates(:,1:2);
%                     gates_pos = gates_pos .* ones(1,1,size(drone_pos,1));
%                     gate_pos = permute(gates_pos,[3,2,1]);
%                     
%                     dist = gate_pos-drone_pos;
%                     dist = squeeze(vecnorm(dist,2,2));
%                     
%                     [~, closest_gate] = min(dist,[],2)
%                     
%                     line_segments = [output.sim_state_pos.Data(1:end-1,1:2), output.sim_state_pos.Data(2:end,1:2)];
%                     
%                     intersect = [gates(:,1:2) + gate_width*[sin(gates(:,4)), -cos(gates(:,4))];
%                                  gates(:,1:2) - gate_width*[sin(gates(:,4)), -cos(gates(:,4))]];
%                         
%                     
%                     
%                     %% check gate completion and time

                    
                end
            end
        end
    end
end

save(filename,'SimData')