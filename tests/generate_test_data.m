clearvars; close all; clc;
addpath('slts_control_sim');
[soln, res] = run_sim('slts_control_sim');
VisualizeTestData(res);
