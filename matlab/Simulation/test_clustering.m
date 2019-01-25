dist_thresh = 20;
intersections=[165,70;
    165,230;
    170,71;
    302,84;
    169,230;
    302,216];

clusters = zeros(size(intersections,1),1);
cluster = 0;
corners = zeros(0,2);
votes = zeros(0,1);
while ~all(clusters)
    cluster = cluster+1;
    % find first unclustered and set as center
    s = find(~clusters,1);
    pos_m = intersections(s,:);
    vote_m = 1;
    % set this point as clustered
    clusters(s) = cluster;
    % find points belonging to this cluster until no more were found (and go_around becomes false)
    for i=1:size(intersections,1)
        if ~clusters(i)
            % go through all unclustered points and calculate distance to cluster center
            pos = intersections(i,:);
            distance = norm(pos-pos_m);
            if distance < dist_thresh
                % recalculate average and votes
                pos_m = (pos_m * vote_m + pos) / (vote_m + 1);
                vote_m = vote_m + 1;
                % set point as clustered
                clusters(i) = cluster;
            end
        end
    end
    % finished. add average to a list of clusters
    corners = [corners; pos_m];
    votes = [votes; vote_m];
end

corners_output = int16(corners);
%     
% while length(corners,1) > 4
%     
%     
    
    
    
    
    
    
    