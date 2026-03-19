function obj_pos = getObjectPosition(sim, objID)
    %GETOBJECTPOSITION Summary of this function goes here
    %   Detailed explanation goes here
    
    obj_pos=nan(1,3);
    try
        obj_pos = sim.getObjectPosition(objID);
        obj_pos = cellfun(@double, obj_pos); % Convert each cell content to double   
    catch
        
    end
 

end

