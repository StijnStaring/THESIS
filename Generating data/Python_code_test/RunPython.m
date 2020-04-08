function out = RunPython(in1, in2)
    out = 0; % Has to be preassigned, otherwise Simulink throws an error
    coder.extrinsic('py.Example.test') % Python functions have to be run extrinsically, meaning no C code generated
    out = py.Example.test(in1,in2);
end