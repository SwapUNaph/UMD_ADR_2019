close all
clc
jet_c = colormap(jet(255));

for i=5
    vid = vout(:,:,i);
%     vid = imgaussfilt(vid, 0.5);
    vid = vid/max(vid(:))*255;
    imshow(vid,jet_c,'InitialMagnification','fit');
    pause(1)
end

% corners = detectHarrisFeatures(vid,);
% hold on;
% plot(corners);

% regionalmax = imregionalmax(vid,8);
% imshow(regionalmax);

return

L = watershed(-vid,8);
% L(~bw) = 0;
rgb = label2rgb(L,'jet',[.5 .5 .5]);
figure
imshow(rgb,'InitialMagnification','fit')
title('Watershed transform of D')

