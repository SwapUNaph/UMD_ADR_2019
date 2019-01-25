corners = [135    25;
   296   212;
    13   212;
   128   212];
   
       Resolution = 1.5*[320, 200];
    f = 0.5*Resolution(2)/tan(0.5*FOV);
    IntrinsicMatrix = [f, 0, 0;
        0, f, 0;
        Resolution/2, 1];
    cameraParams = cameraParameters('IntrinsicMatrix',IntrinsicMatrix); 
    
    %% calculate extrinsics
    worldPoints = gate_size/2*[1 1;
        1 -1;
        -1 1;
        -1 -1];
    
    corners=double(corners);

   
    angles = zeros(6,1);
    k = 1;
    for m=1:3
        for n=m+1:4
            angles(k)=atan2(corners(m,2)-corners(n,2),corners(m,1)-corners(n,1));
            k = k+1;            
        end
    end
    
    angles(angles<=0) = angles(angles<=0)+pi;
    
    a = zeros(2,1);
    a = hist(angles,unique(angles));
    if max(a)>2
        disp('three occurences')
    end
    
    
    [rMat,tVec] = extrinsics(corners,worldPoints,cameraParams);
    
    
    drone_pos=[1,1,1];
    %% drone orientation
    dRg = eul2rotm([1,0,0]);
    
    %% convert gate pos
    % tVec = right down front, drone was front left up
    dRc = eul2rotm([pi/2,-pi/2,0],'ZYX');
    cRd = dRc^-1;
    cRg = dRg*cRd;
    rel= cRg*tVec';
    gate_pos_global = drone_pos + rel;
    
    % rel = -dRs*tvec';
    % gate_pos_global = drone_pos + rel';
    
    %% convert gate orientation
    rot = squeeze(rMat);
    extra = eul2rotm([0,pi/2,0],'ZYX');
    gate_rot = dRg*cRd*rot^-1*extra;
    gate_eul_global = rotm2eul(gate_rot)';