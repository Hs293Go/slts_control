clearvars; close all; clc;
addpath('slts_control_matlab');
[soln, res] = SimulationRunner('slts_control_matlab');
VisualizeTestData(res);
