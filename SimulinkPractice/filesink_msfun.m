%% S-function entry point
function filesink_msfun(block)
setup(block);

%% First required callback: setup
function setup(block)

block.NumDialogPrms =  1; % Number of parameters
block.NumInputPorts = 1; % Number of inports
block.NumOutputPorts = 0; %Number of outputs 

% Set input port properties as inherited 
block.SetPreCompInpPortInfoToDynamic; 

% Override some properties: scalar real input 
block.InputPort(1).Dimensions = 1;
block.InputPort(1).DatatypeID = 0; %double 
block.InputPort(1).Complexity = 'Real';
block.InputPort(1).SamplingMode = 'Inherited';
block.InputPort(1).DirectFeedthrough = 1; 

% Set the smaple time and the offset time.
%  [0 offset]               : Continuous sample rate
%  [positive_num_offset]    : Discrete sample rate  
%  [-1, 0]                  : Inherited sample rate  
%  [-2, 0]                  : Variable sample rate 
block.SampleTimes = [-1 0];

% Specify the block simStateCompliance. The allowed values are:
%  'UnknownSimState',   < The default setting; warn and assume
%  DefaultSimState
%   'DefaultSimState',  < Same sim state as a buil-in block
%   'HasNoSimState',    < No sim state
%   'CustomSimState',   < Has GetSimState and SetSimState methods 
%   'DisallowSimState,' < Error out when saving or restoring the model sim
%   state 
block.SimStateCompliance = 'HasNoSimState';

block.RegBlockMethod('Outputs', @Outputs); % Required

%% Second required callback: Outputs
function Outputs(block)
% open the file as write-only
fid = fopen(block.DialogPrm(1).Data, 'w');
%print input port value to file 
fprintf(fid, '$f', block.InputPort(1).Data);
% close the file 
fclose(fid); 