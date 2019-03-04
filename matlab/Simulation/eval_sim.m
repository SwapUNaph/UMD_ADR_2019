clc
close all
clear

gate_width = 0.7;
gate_r = 0.025;
drone_r = 0.2;

filename = 'Sim4.mat';
SimxD=[3,3,500];
load(filename);

SimResult = zeros(size(SimData,1),10);

for sim = 1:size(SimData,1)
    if isequal(SimData{sim,1},[])
        SimResult(sim,7) = 1;
    else

    % SimData(i,1) = num2cell(output.sim_idx.Time,1);
    % SimData(i,2) = num2cell(output.sim_idx.Data,1);
    % SimData(i,3) = num2cell(output.sim_state_pos.Time,1);
    % SimData(i,4) = num2cell(output.sim_state_pos.Data,[1,2]);
    % SimData(i,5) = num2cell(output.sim_gates.Data(:,:,1),[1,2]);
    sim_idx_Time = SimData{sim,1};
    sim_idx_Data = SimData{sim,2};
    sim_state_pos_Time = SimData{sim,3};
    sim_state_pos_Data = SimData{sim,4};
    sim_gates = SimData{sim,5};

    %% check collision
    % for every line segment check distance to all circles
    drone_pos_xy = sim_state_pos_Data(:,1:2);
    gates = sim_gates(:,:,1);
    drone_pos_xy_p = drone_pos_xy .* ones(1,1,size(gates,1));
    gates_pos_xy = gates(:,1:2);
    gates_pos_xy = gates_pos_xy .* ones(1,1,size(drone_pos_xy,1));
    gates_pos_xy = permute(gates_pos_xy,[3,2,1]);

    dist_g = gates_pos_xy-drone_pos_xy_p;
    dist_g = squeeze(vecnorm(dist_g,2,2));

    [~, closest_gate] = min(dist_g,[],2);
    gates_lr = cat(3, gates(:,1:2) + gate_width*[sin(gates(:,4)), -cos(gates(:,4))], ...
                      gates(:,1:2) - gate_width*[sin(gates(:,4)), -cos(gates(:,4))]);

    closest_gate_lr = gates_lr(closest_gate,:,:);
    % closest_gate_lr = closest_gate_lr(2:end,:,:);

    % line_segments = [drone_pos_xy(1:end-1,:), drone_pos_xy(2:end,:)];

    drone_pos_xy_p = drone_pos_xy .* ones(1,1,2);
    dist_lr = min(vecnorm(drone_pos_xy_p - closest_gate_lr,2,2),[],3);
    min_dist = min(dist_lr);
    SimResult(sim,1) = min_dist;
    SimResult(sim,2) = min_dist > gate_r+drone_r;
    
    
    %% check gate passing in between posts
    times = sim_idx_Time(diff(sim_idx_Data)~=0)*ones(1,2)+[0 1];
    times = permute(times.*ones(1,1,size(sim_state_pos_Time,1)),[3,2,1]);
    [~,idx] = min(abs(sim_state_pos_Time.*ones(size(sim_state_pos_Time,1),2,size(times,3))-times));
    
    trackpart = zeros(500,3,size(idx,3));
%     plot(sim_state_pos_Data(:,1),sim_state_pos_Data(:,2),'-')
%     hold on
    for gate=1:size(idx,3)
        trackpart(1:(idx(1,2,gate)+1-idx(1,1,gate)),:,gate) = sim_state_pos_Data(idx(1,1,gate):idx(1,2,gate),:);
%         data=trackpart{1,gate};
%         plot(data(:,1),data(:,2),'x-')
%         hold on
    end
    gates_rep = permute(gates(1:size(idx,3),:).*ones(1,1,500),[3,2,1]);
    [d,idx] = min(vecnorm(trackpart(:,1:2,:) - gates_rep(:,1:2,:),2,2));
    inside=all(d<0.7);
    idx = squeeze(idx);
    closest = zeros(size(trackpart,3),3);
    for gate=1:size(closest,1)
        closest(gate,:) = trackpart(idx(gate),:,gate);
    end
    
    
    

    %% check gate completion and time
    SimResult(sim,3) = max(sim_idx_Data);
    SimResult(sim,4) = (max(sim_idx_Data)==7 );%&& sim_idx_Data(end) == 0);
    SimResult(sim,5) = sim_idx_Time(end);
    SimResult(sim,6) = 1;
    SimResult(sim,7) = inside;
    
    end
end

Sim_xD_dist = reshape(SimResult(1:prod(SimxD),2),SimxD);
Sim_xD_gate = reshape(SimResult(1:prod(SimxD),4),SimxD);
Sim_xD_sim  = reshape(SimResult(1:prod(SimxD),6),SimxD);
Sim_xD_succ = Sim_xD_dist & Sim_xD_gate;
Sim_xD_time = reshape(SimResult(1:prod(SimxD),5),SimxD);
Sim_xD_time_succ = Sim_xD_time;
Sim_xD_time_succ(Sim_xD_succ ~= 1) = 0;
Sim_xD_succ(Sim_xD_time_succ == 100) = 0;
Sim_xD_time_succ(Sim_xD_time_succ == 100) = 0;

Sim_xD_dist_sum = sum(Sim_xD_dist,3);
Sim_xD_gate_sum = sum(Sim_xD_gate,3);
Sim_xD_sim_sum = sum(Sim_xD_sim,3);
Sim_xD_succ_sum = sum(Sim_xD_succ,3);
Sim_xD_time_succ_av = sum(Sim_xD_time_succ,3)./Sim_xD_succ_sum

Sim_xD_dist_sum./Sim_xD_sim_sum;
Sim_xD_gate_sum./Sim_xD_sim_sum;
Sim_xD_succ_sum./Sim_xD_sim_sum;

save(filename,'SimData','SimResult')

% 1: min dist
% 2: min dist ok?
% 3: maximum gate
% 4: reached end?
% 5: maximum time
% 6: simulated yes
% 7: not simulated