clc;
close all;
clear all;

A = csvread('fixed_current_points.csv')';
B = csvread('reference_points.csv')';

[regParams, Bfit, ErrorStats] = absor(A, B, 'doScale', true, 'doTrans', true);

%  A: a 2xN or 3xN matrix whos columns are the coordinates of N source points.
%  B: a 2xN or 3xN matrix whos columns are the coordinates of N target points.

disp(ErrorStats);

filename = 'cloud.ply';
pc = readply(filename);

points = pc(:, 1:3)';

regParams.t(1) = regParams.t(1) - 3;    % proper adjustement on x and z
regParams.t(3) = regParams.t(3) - 3;

new_points = regParams.s * regParams.R * points + repmat(regParams.t, 1, length(points));

write_ply('new_cloud.ply', new_points, pc(:, 4:6)');

