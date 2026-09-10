% Preserve the supplied reference before main.m overwrites it
assert(~isfile('offline_design_reference.mat'), ...
    'Reference backup already exists; do not overwrite it.');
copyfile('offline_design.mat', 'offline_design_reference.mat');

% Capture solver diagnostics
diary('baseline_rompc_log.txt');
main
diary off