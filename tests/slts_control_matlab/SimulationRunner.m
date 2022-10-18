function [soln, res] = SimulationRunner(save_path)
    model = 'test_controller';
    in = Simulink.SimulationInput(model);
    in = in.setModelParameter('SimulationMode', 'accelerator');
    in = in.setModelParameter('StopTime', '60');
    in = in.setModelParameter('SolverType', 'Fixed-step');
    in = in.setModelParameter('FixedStep', '0.01');
    in = in.setModelParameter('SolverName', 'euler');

    soln = sim(in);
    tout = soln.tout;
    yout = soln.yout;

    json = struct('tout', struct('size', size(tout), 'value', tout));
    res = struct('tout', tout);

    for it = reshape(cellfun(@string, yout.getElementNames), 1, [])
        val = squeeze(yout.get(it).Values.Data);

        % Convert dataset to regular struct
        res.(it) = val;

        % Convert dataset to json, each data item consisting of a size and
        % 1D data array
        sz = size(val);

        if isrow(val)
            sz = sz(end:-1:1);
        end

        json.(it) = struct('size', sz, 'value', reshape(val, [], 1));
    end
    if nargin == 1 && ~isempty(save_path)
        JsonDump(sprintf('%s/test_controller.json', save_path), json);
    end
end
