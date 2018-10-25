
dirlist = [];
dirlist = dir('*_rtw');
dirlist = [dirlist; dir('*lprj')];
dirlist = [dirlist; dir('*texture')];


filelist1 = dir('*.mexw64');
filelist2 = dir('*.o');
filelist3 = dir('*.original');
filelist4 = dir('*.autosave');
% filelist5 = dir('*.exe');
filelist = [filelist1; filelist2; filelist3; filelist4];


for i = 1:size(dirlist,1)
    rmdir(dirlist(i).name,'s');
end

for i = 1:size(filelist,1)
    delete(filelist(i).name);
end

close all; clear all; clc