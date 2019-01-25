












% 
% function plot3d(drone_vel, drone_pos, drone_ori, WP_vel, WP_yaw, WP_pos, gate_pos_global, gate_ori_global, current_spline, std_dev, WP_list)
% coder.extrinsic('delete')
% coder.extrinsic('ellipsoid')
% coder.extrinsic('viscircles')
% coder.extrinsic('quiver3')
% 
% children = get(gca, 'children');
% delete(children(1:19));
% legends=zeros(50,1);
% 
% gRb = eul2rotm([drone_ori(3),drone_ori(2),drone_ori(1)]);
% length = 1;
% x_bebop = gRb*[length;0;0];
% y_bebop = gRb*[0;length;0];
% z_bebop = gRb*[0;0;length];
% 
% 
% legends(1)=plot3(drone_pos(1),drone_pos(2),drone_pos(3),'m.','MarkerSize',7);
% legends(2)=plot3(drone_pos(1),drone_pos(2),0,'k.','MarkerSize',5);
% 
% 
% % spline
% d = (0:0.1:1)';
% P0 = current_spline(1,:);
% P1 = current_spline(2,:);
% P2 = current_spline(3,:);
% spline_pos = (1-d)*[1,1,1,1].*((1-d)*P0+d*P1)+d*[1,1,1,1].*((1-d)*P1 + d*P2);
% legends(3)=plot3(spline_pos(:,1),spline_pos(:,2),spline_pos(:,3),'-y','LineWidth',2);
% 
% % drone pos
% legends(4)=plot3(drone_pos(1),drone_pos(2),drone_pos(3),'kx');
% legends(5)=plot3(drone_pos(1)+[0 drone_vel(1)],drone_pos(2)+[0 drone_vel(2)],drone_pos(3)+[0 drone_vel(3)],'k-','LineWidth',2);
% 
% % drone ori
% legends(6)=plot3(drone_pos(1)+[0 x_bebop(1)],drone_pos(2)+[0 x_bebop(2)],drone_pos(3)+[0 x_bebop(3)],'r-');
% legends(7)=plot3(drone_pos(1)+[0 y_bebop(1)],drone_pos(2)+[0 y_bebop(2)],drone_pos(3)+[0 y_bebop(3)],'g-');
% legends(8)=plot3(drone_pos(1)+[0 z_bebop(1)],drone_pos(2)+[0 z_bebop(2)],drone_pos(3)+[0 z_bebop(3)],'b-');
% 
% % nav pt pos, vel, yaw
% legends(9)=plot3(WP_pos(1),WP_pos(2),WP_pos(3),'ko');
% legends(10)=plot3(WP_pos(1)+[0 WP_vel(1)],WP_pos(2)+[0 WP_vel(2)],WP_pos(3)+[0 WP_vel(3)],'b-','LineWidth',2);
% % plot3(WP_pos(1)+[0 0.5*cos(WP_yaw)],WP_pos(2)+[0 0.5*sin(WP_yaw)],WP_pos(3)+[0 0],'r-','LineWidth',1)
% 
% % WP current pos + ori
% if isequal(gate_ori_global, zeros(1,3)) || isequal(gate_ori_global, zeros(3,1))
%     % no gate info
%     legends(11)=plot3(0,0,0,'.');
%     legends(12)=plot3(0,0,0,'.');
% %     plot3(0,0,0,'.')
% %     plot3(0,0,0,'.')
% else   
%     % pos
%     legends(11)=plot3(gate_pos_global(1), gate_pos_global(2), gate_pos_global(3),'gx');
%     % ori
%     gate_rot = eul2rotm(gate_ori_global);
%     length = 1;
%     x_gate = gate_rot*[length;0;0];
% %     y_gate = gate_rot*[0;length;0];
% %     z_gate = gate_rot*[0;0;length];
%     legends(12)=plot3(gate_pos_global(1) + [0 x_gate(1)],gate_pos_global(2) + [0 x_gate(2)],gate_pos_global(3) + [0 x_gate(3)],'g-','LineWidth',1);
% %     plot3(gate_pos_global(1) + [0 y_gate(1)],gate_pos_global(2) + [0 y_gate(2)],gate_pos_global(3) + [0 y_gate(3)],'g-','LineWidth',1)
% %     plot3(gate_pos_global(1) + [0 z_gate(1)],gate_pos_global(2) + [0 z_gate(2)],gate_pos_global(3) + [0 z_gate(3)],'b-','LineWidth',1)
% end
% 
% % all WP
% legends(13)=plot3(WP_list(:,1),WP_list(:,2),WP_list(:,3),'ob');
% legends(14)=quiver3(WP_list(:,1),WP_list(:,2),WP_list(:,3),cos(WP_list(:,4)),sin(WP_list(:,4)),zeros(4,1),0,'b');;
% 
% % WP average pos
% std_dev = std_dev*3;
% legends(15)=plot3(P2(1),P2(2),P2(3),'ro');
% legends(16)=quiver3(P2(1),P2(2),P2(3),cos(P2(4)),sin(P2(4)),0,0,'r','MaxHeadSize',1);
% legends(17)=quiver3(P2(1),P2(2),P2(3),cos(P2(4)+std_dev(4)),sin(P2(4)+std_dev(4)),0,0.6,'r:','ShowArrowHead','off','LineWidth',1);
% legends(18)=quiver3(P2(1),P2(2),P2(3),cos(P2(4)-std_dev(4)),sin(P2(4)-std_dev(4)),0,0.6,'r:','ShowArrowHead','off','LineWidth',1);
% 
% % WP std dev
% % [ellx,elly,ellz] = ellipsoid(P2(1),P2(2),P2(3),3*std_dev(1),3*std_dev(2),3*std_dev(3),20);
% % surf(ellx,elly,ellz,ellz,'FaceAlpha',0.5)
% r=linspace(0,2*pi,4*4);
% legends(19)=plot3(P2(1)+std_dev(1)*sin(r),P2(2)+std_dev(2)*cos(r),P2(3)+0*r,':r','LineWidth',1);
% % viscircles(P2(1:2),max(std_dev(1:3)),':','LineWidth',1)
% 
% legend(legends(1:19),'test')