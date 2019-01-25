close all
clc

figure('units','normalized','outerposition',[0 0 0.5 1])
hold on
grid on
axis equal

for i=1:11
    plot3(0,0,2,'m.')
end

axis([-0.5,4,-2,3.5,1.5,2.5])
view(-90,90) % top view
xlabel('x')
ylabel('y')
zlabel('z')

drone_ori_a = drone_ori.data;
drone_pos_a = drone_pos.data;
rmat_a = rMat.data;
tvec_a = tVec.data;

for i=[1:90, 650:length(drone_pos_a)]
    children = get(gca, 'children');
    delete(children(1:7));
    
    % drone position
    plot3(drone_pos_a(i,1),drone_pos_a(i,2),drone_pos_a(i,3),'.m')
    
    % drone rotation
    drone_rot = axang2rotm(drone_ori_a(i,:))*eul2rotm([pi/2, pi/2, 0]);
    l = 1;
    x_drone = drone_rot*[l;0;0];
    y_drone = drone_rot*[0;l;0];
    z_drone = drone_rot*[0;0;l];
    plot3(drone_pos_a(i,1) + [0 x_drone(1)],drone_pos_a(i,2) + [0 x_drone(2)],drone_pos_a(i,3) + [0 x_drone(3)],'r-','LineWidth',2)
    plot3(drone_pos_a(i,1) + [0 y_drone(1)],drone_pos_a(i,2) + [0 y_drone(2)],drone_pos_a(i,3) + [0 y_drone(3)],'g-','LineWidth',2)
    plot3(drone_pos_a(i,1) + [0 z_drone(1)],drone_pos_a(i,2) + [0 z_drone(2)],drone_pos_a(i,3) + [0 z_drone(3)],'b-','LineWidth',2)
    
    % gate position
    dRc = eul2rotm([pi/2,-pi/2,0],'ZYX');
    cRd = dRc^-1;
    
    tr = squeeze(tvec_a(:,:,i))';
    drone_pos_a(i,:)';
    gate_pos_rel = drone_rot*cRd*tr;
    gate_pos_gl = drone_pos_a(i,:)'+gate_pos_rel;
    plot3(gate_pos_gl(1),gate_pos_gl(2),gate_pos_gl(3),'ro')
    
    % gate orientation
    rot = squeeze(rmat_a(:,:,i));
    extra = eul2rotm([0,pi/2,0],'ZYX');
    gate_rot = ((drone_rot*cRd)*rot^-1)*extra;
    l = 1;
    x_gate = gate_rot*[l;0;0];
    y_gate = gate_rot*[0;l;0];
    z_gate = gate_rot*[0;0;l];
    plot3(gate_pos_gl(1) + [0 x_gate(1)],gate_pos_gl(2) + [0 x_gate(2)],gate_pos_gl(3) + [0 x_gate(3)],'r-','LineWidth',2)
    plot3(gate_pos_gl(1) + [0 y_gate(1)],gate_pos_gl(2) + [0 y_gate(2)],gate_pos_gl(3) + [0 y_gate(3)],'g-','LineWidth',2)
    plot3(gate_pos_gl(1) + [0 z_gate(1)],gate_pos_gl(2) + [0 z_gate(2)],gate_pos_gl(3) + [0 z_gate(3)],'b-','LineWidth',2)
    
    
    pause(0.0001)
end

