% clearvars; close all; clc;
addpath('models');
model = 'test_controller';
in = Simulink.SimulationInput(model);
in = in.setModelParameter('SimulationMode', 'accelerator');
in = in.setModelParameter('StopTime', '100');
in = in.setModelParameter('SolverType', 'Fixed-step');
in = in.setModelParameter('SolverName', 'euler');

% soln = sim(in);

pld = DatasetToJStruct(soln, ["pld_abs_pos", "pld_rel_pos", "pld_abs_vel", "pld_rel_vel", "act_force"]);

fid = fopen('test_controller.json', 'w');
fprintf(fid, '%s', jsonencode(pld));
fclose(fid);

function json = DatasetToJStruct(ds, fields)
    
    tout = ds.tout;
    yout = ds.yout;
    json = struct('tout', struct('size', size(tout), 'value', tout));
    for it = fields
        val = squeeze(yout.get(it).Values.Data).';
        sz = size(val);
        val = reshape(val, [], 1);
        json.(it) = struct('size', sz, 'value', val);
    end

end
