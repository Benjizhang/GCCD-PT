% initialize the GCCD-PT library
% Run it when you need to use the GCCD-PT.
%
% Author: @ Benji Z. Zhang (zzqing@connect.hku.hk)
% Date:2022/07

clc; clear; close all;

%% go to the home path
file = mfilename('fullpath');
[filepath,name,ext] = fileparts(file);
cd(filepath)

%% add library to the environment path
folderName = cd;
p = genpath(folderName);
p_ls = strsplit(p, ';');

% clean the .git path
pat = '.git';
nogit_path = cell(1);
k = 1;
for i = 1:length(p_ls)
    cur_path = p_ls{i};    
    if ~contains(cur_path,pat) && ~isempty(cur_path)
       nogit_path{k} = cur_path;
       k = k+1;
    end
end

% add to the environment path
addpath(nogit_path{:})

disp('---- GCCD-PT Initialized Successfully ----')
disp('[Tip] Start GCCD-PT from the script folder.')
