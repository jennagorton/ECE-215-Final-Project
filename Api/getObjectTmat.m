function [T] = getObjectTmat(sim, refID, objID)
%GETOBJECT_TMAT  Return 4x4 homogeneous transform from CoppeliaSim
%
%   T = getObject_Tmat(sim, objID)
%   T = getObject_Tmat(sim, objID, refID)
%
%   sim   : CoppeliaSim Remote API sim object
%   refID : reference frame handle (world = -1)
%   objID : object handle
%
%   T     : 4x4 homogeneous transformation matrix
    
    T = nan(4,4);

    if nargin ~= 3 
        fprintf("Wrong Input");
        return;
    end

    % Get raw pose (1x12 cell)
    Mcell = sim.getObjectMatrix(objID, refID);

    % Force every element in the cell array to be a double
    Mcell = cellfun(@double, Mcell, 'UniformOutput', false);

    M = cell2mat(Mcell);   % 1x12 double
    
    % Rotation matrix (columns are object axes)
    R = [ M(1)  M(2)  M(3);
          M(5)  M(6)  M(7);
          M(9)  M(10)  M(11) ];

    % Translation vector
    p = [ M(4); M(8); M(12) ];

    % Homogeneous transform
    T = [R p;
         0 0 0 1];
end
