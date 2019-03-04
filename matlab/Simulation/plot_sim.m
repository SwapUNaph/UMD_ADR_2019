close all 
clc

gate_width = 0.7;
gate_r = 0.025;
drone_r = 0.2;


filename = 'Sim2.mat';
SimxD=[4,4,132];
load(filename);

gate_width = 0.7;
gate_thick = 0.05;

height = 2.3;
flightplan = [ 4.348, -0.278, height, deg2rad(  0);
              10.805, -0.278, height, deg2rad(  0);
              12.146, -3.775, height, deg2rad(-90);
               8.769, -6.876, height, deg2rad(180);
               2.593, -6.764, height, deg2rad(180);
               2.657, -3.652, height, deg2rad(  0);
               7.172, -3.648, height, deg2rad( 90);
               4.348, -0.278, height, deg2rad(180)];

% inside
do_in = SimResult(:,6) == 1 & SimResult(:,7) == 0 & SimResult(:,2) == 1 & SimResult(:,4) == 1;

% dist
do_dist = SimResult(:,6) == 1 & SimResult(:,2) == 0;

% end
do_end = SimResult(:,6) == 1 & SimResult(:,4) == 0;

for sim = 1:size(SimResult,1)
    if true%do_in(sim)
        close all
        figure('units','normalized','outerposition',[0 0 0.5 1])

        sim_idx_Time = SimData{sim,1};
        sim_idx_Data = SimData{sim,2};
        sim_state_pos_Time = SimData{sim,3};
        sim_state_pos_Data = SimData{sim,4};
        sim_gates = SimData{sim,5};
        
        %% calc takeoff and flightplan
        takeoff = sim_state_pos_Data(1,1:2);
        flightplan_r = ones(size(flightplan,1),1)*takeoff + flightplan(:,1:2);
        
        hold on
        for gate = 1:size(flightplan_r,1)
            p = flightplan_r(gate,:);
            ps = [-p(2), p(1)]+[ -gate_width*cos(flightplan(gate,4))-gate_thick*sin(flightplan(gate,4)),gate_thick*cos(flightplan(gate,4))+gate_width*sin(flightplan(gate,4));
                           gate_width*cos(flightplan(gate,4))+gate_thick*sin(flightplan(gate,4)),gate_thick*cos(flightplan(gate,4))+gate_width*sin(flightplan(gate,4));
                         gate_width*cos(flightplan(gate,4))+gate_thick*sin(flightplan(gate,4)),-gate_thick*cos(flightplan(gate,4))-gate_width*sin(flightplan(gate,4));
                         -gate_width*cos(flightplan(gate,4))-gate_thick*sin(flightplan(gate,4)),-gate_thick*cos(flightplan(gate,4))-gate_width*sin(flightplan(gate,4))];
            fill(ps(:,2),-ps(:,1),'g','EdgeColor','none')
            l=1;
            quiver(p(1),p(2),l*cos(flightplan(gate,4)),l*sin(flightplan(gate,4)),'g','LineWidth',3,'MaxHeadSize',10)
            text(p(1)+sin(flightplan(gate,4)),p(2)+cos(flightplan(gate,4)),num2str(gate),'FontSize',15)
        end

        
        %% calculate gates
        gates = sim_gates(:,:,1);
        gates_lr = cat(3, gates(:,1:2) + gate_width*[sin(gates(:,4)), -cos(gates(:,4))], ...
                          gates(:,1:2) - gate_width*[sin(gates(:,4)), -cos(gates(:,4))]);

        %% plot
        
        hold on
        
        for gate = 1:size(flightplan_r,1)
            p = gates(gate,:);
            
            ps = [-p(2), p(1)]+[ +gate_width*cos(gates(gate,4))+gate_thick*sin(gates(gate,4)),+gate_width*sin(gates(gate,4))-gate_thick*cos(gates(gate,4));
                           +gate_width*cos(gates(gate,4))-gate_thick*sin(gates(gate,4)),+gate_width*sin(gates(gate,4))+gate_thick*cos(gates(gate,4));
                         -gate_width*cos(gates(gate,4))-gate_thick*sin(gates(gate,4)),-gate_width*sin(gates(gate,4))+gate_thick*cos(gates(gate,4));
                         -gate_width*cos(gates(gate,4))+gate_thick*sin(gates(gate,4)),-gate_width*sin(gates(gate,4))-gate_thick*cos(gates(gate,4))];
            fill(ps(:,2),-ps(:,1),'r','FaceColor',[1,0.5,0],'EdgeColor','none')
            l=1;
            quiver(p(1),p(2),l*cos(gates(gate,4)),l*sin(gates(gate,4)),'k','Color',[1 0.5 0],'LineWidth',3,'MaxHeadSize',10)
        end
        
        

        plot(sim_state_pos_Data(:,1),sim_state_pos_Data(:,2),'-b')
        plot(sim_state_pos_Data([1 end],1),sim_state_pos_Data([1 end],2),'xk','MarkerSize',15)
        text(sim_state_pos_Data(1,1)+0.3,sim_state_pos_Data(1,2)+0.3,'start','FontSize',15)
        text(sim_state_pos_Data(end,1)+0.3,sim_state_pos_Data(end,2)+0.3,'finish','FontSize',15)
        
        axis equal
        grid on
        
        xlabel('x[m]')
        ylabel('y[m]')
        pause('on');
        pause;
        
    end
end