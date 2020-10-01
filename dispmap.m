function [dmap, unrel] = dispmap(subject,left,right,mask)

mask=rgb2gray(mask);left=rgb2gray(left);right=rgb2gray(right);

switch subject
    case 'S1' % Subject 1
        disparityRange = [276,340];
        dmap = disparity(left,right,'DisparityRange',disparityRange, ...
            'UniquenessThreshold',5,'DistanceThreshold',15);
        dmap(imcomplement(mask > 0)) = 0; %Mask background
		%Smoothing and Extend image at boundaries
        dmap = medfilt2(dmap, [50 50],'symmetric');  
    case 'S2' % Subject 2
        disparityRange = [292 372];
        dmap = disparity(left,right,'DisparityRange',disparityRange, ...
            'UniquenessThreshold',1,'DistanceThreshold',5);
        dmap(imcomplement(mask > 0)) = 0;
        dmap = medfilt2(dmap, [100 100], 'symmetric');
    case 'S4' % Subject 4
        disparityRange = [346 426];
        dmap = disparity(left,right,'DisparityRange',disparityRange, ...
            'UniquenessThreshold',5,'DistanceThreshold',10);
        dmap(imcomplement(mask > 0)) = 0;
        dmap = medfilt2(dmap, [100 100],'symmetric');
end

dmap = imfill(dmap,'holes'); %Remove holes
figure;imshow(dmap,disparityRange);
colormap(gca,jet);colorbar; %Depth colors

%Unreliable map
unrel = ones(size(dmap));
unrel(find(dmap~=0)) = 0;
unrel(find(dmap==-realmax('single'))) = 1; %Remove unreliable disparity

end
