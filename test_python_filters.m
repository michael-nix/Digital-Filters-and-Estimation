% Short test file to practice making Python code work in MATLAB when
% dealing with some of the filters that we've been discussing so far.
clear classes; %#ok<CLCLS>

py_filters_module = py.importlib.import_module('example_python_filters');
py.importlib.reload(py_filters_module);

% test Butterworth filter
filter = cell(py.example_python_filters.py_getbutter(0.1));
alphab = double(filter{1});
betab = double(filter{2});

% test notch filter
filter = cell(py.example_python_filters.py_getnotch(0.1));
alphan = double(filter{1});
betan = double(filter{2});

% test extended Kalman filter
z = py.numpy.zeros(int32([3, 1]));
x_est = py.numpy.ones(int32([9, 1]));
P_est = py.numpy.eye(int32(9), int32(9));

estimates = cell(py.example_python_filters.py_exampleEKF(z, 1, x_est, P_est));
x_est = double(estimates{1});
P_est = double(estimates{2});