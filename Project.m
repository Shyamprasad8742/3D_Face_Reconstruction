clear variables
close all
clc

%% Reading image
subject1=imageSet('data/subject1');
subject2=imageSet('data/subject2');
subject4=imageSet('data/subject4');
%% Select subject
%sub=subject1;
%sub=subject2;
sub=subject4;
img_L = im2double(read(sub,1));
img_M = im2double(read(sub,2));
img_R = im2double(read(sub,3));

%% Stereo Camera Calibration
%% Processed using Stereo camera calibrator application
%save stereoParamsC1 stereoParamsC1LM stereoParamsC1MR
%save stereoParamsC2 stereoParamsC2LM stereoParamsC2MR
load stereoParamsC1.mat
%load stereoParamsC2.mat
figure(1);showExtrinsics(stereoParamsC1LM)
figure(2);showExtrinsics(stereoParamsC1MR)
%figure(3);showExtrinsics(stereoParamsC2LM)
%figure(4);showExtrinsics(stereoParamsC2MR)

%% Mask of the subject using k-means clustering
%% Processed using kmeans function created
%[mask_L] = k_means(img_L);
mask_L1=cat(3,mask_L,mask_L,mask_L);
mask_L2 = img_L;
mask_L2(imcomplement(mask_L1))=0;
figure(5);imshow(mask_L,[]);
%imwrite(mask_L,'maskS4L.jpg')
figure(6);imshow(mask_L2,[]);
%imwrite(mask_L2,'faceS4L.jpg')

%[mask_M] = k_means(img_M);
mask_M1=cat(3,mask_M,mask_M,mask_M);
mask_M2 = img_M;
mask_M2(imcomplement(mask_M1))=0;
figure(7);imshow(mask_M,[]);
%imwrite(mask_M,'maskS4M.jpg')
figure(8);imshow(mask_M2,[]);
%imwrite(mask_M2,'faceS4M.jpg')

%[mask_R] = k_means(img_R);
mask_R1=cat(3,mask_R,mask_R,mask_R);
mask_R2 = img_R;
mask_R2(imcomplement(mask_R1))=0;
figure(9);imshow(mask_R,[]);
%imwrite(mask_R,'maskS4R.jpg')
figure(10);imshow(mask_R2,[]);
%imwrite(mask_R2,'faceS4R.jpg')

%save maskS1 mask_L mask_M mask_R
%save maskS2 mask_L mask_M mask_R
%save maskS4 mask_L mask_M mask_R

%% Once mask extracted, saved and used for future executions
%load maskS1.mat
%load maskS2.mat
load maskS4.mat

%% Stereo rectification
% Rectify the images of the subject
[img_L_rec,img_LM_rec]=rectifyStereoImages(img_L,img_M, ...
    stereoParamsC1LM,'OutputView','full');
figure(11);imshow(img_L_rec,[]);
%imwrite(img_L_rec,'recfullL.jpg')
figure(12);imshow(img_LM_rec,[]);
%imwrite(img_LM_rec,'recfullLM.jpg')
    
[img_MR_rec,img_R_rec]=rectifyStereoImages(img_M,img_R, ...
    stereoParamsC1MR, 'OutputView','full');
figure(13);imshow(img_MR_rec,[]);
%imwrite(img_MR_rec,'recfullMR.jpg')
figure(14);imshow(img_R_rec,[]);
%imwrite(img_R_rec,'recfullR.jpg')


% Rectify the masked images of the subject
[img_L_recm,img_LM_recm]=rectifyStereoImages(mask_L2, mask_M2, ...
    stereoParamsC1LM,'OutputView','full');
figure(15);imshow(img_L_recm,[]);
%imwrite(img_L_recm,'recmaskL.jpg')
figure(16);imshow(img_LM_recm,[]);
%imwrite(img_LM_recm,'recmaskLM.jpg')
        
[img_MR_recm,img_R_recm]=rectifyStereoImages(mask_M2,mask_R2, ...
    stereoParamsC1MR, 'OutputView','full');
figure(17);imshow(img_MR_recm,[]);
%imwrite(img_MR_recm,'recmaskMR.jpg')
figure(18);imshow(img_R_recm,[]);
%imwrite(img_R_recm,'recmaskR.jpg')

%% Stereo matching
%% Processed by Registration Estimator application
[stmatch_LM]= SMLM4(img_L_recm, img_LM_recm);
%print -r150 -dpng SMS4LM.png
[stmatch_MR]= SMMR4(img_MR_recm, img_R_recm);
%print -r150 -dpng SMS4MR.png

%% Disparity map
%% Processed using dispmap function created
[dmap_LM,unrel_LM]=dispmap('S4',img_L_rec,img_LM_rec,img_L_recm);
%print -r150 -dpng dispLM.png
[dmap_MR,unrel_MR]=dispmap('S4',img_MR_rec,img_R_rec,img_MR_recm);
%print -r150 -dpng dispMR.png

%% 3D point clouds
% Create point clouds
scene3d_LM = reconstructScene(dmap_LM,stereoParamsC1LM);
scene3d_MR = reconstructScene(dmap_MR,stereoParamsC1MR);
pc_LM = pointCloud(scene3d_LM);pc_MR = pointCloud(scene3d_MR);
dn_LM = pcdenoise(pc_LM);dn_MR = pcdenoise(pc_MR);
ds_LM = pcdownsample(dn_LM,'nonuniformGridSample',6);
ds_MR = pcdownsample(dn_MR,'nonuniformGridSample',6);
[tform,mreg,rmse]= pcregistericp(ds_MR, ds_LM,'Extrapolate',true);
ptcloudout=pcmerge(ds_LM,mreg,1);
figure(19);pcshow(ptcloudout);
view([19 -90]);
%print -r150 -dpng pointcloud.png


%% Create 3D meshes from point clouds
% Processed using function from UT syllabi
mesh_LM = mesh_3D(dmap_LM,scene3d_LM,unrel_LM,img_L_recm);
%xlim([-209 17]);ylim([-250 200]);zlim([428 800]);view([126 59])
mesh_MR = mesh_3D(dmap_MR,scene3d_MR,unrel_MR,img_MR_recm);
%xlim([100 300]);ylim([-290 250]);zlim([350 650]);view([-139 68])