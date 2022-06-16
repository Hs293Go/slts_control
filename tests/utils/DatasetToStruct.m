function json = DatasetToStruct(ds, fields)
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