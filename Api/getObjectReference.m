function [obj_ID] = getObjectReference(sim, objectName)
    %% Summary of this function goes here
    %   This fucntion will return the reference of the 
    if ~startsWith(objectName, '/')
        objectName = strcat('/', objectName);
    end
    obj_ID = nan;
    try
        obj_ID= sim.getObject(objectName);        
    catch        
    end
    

end

