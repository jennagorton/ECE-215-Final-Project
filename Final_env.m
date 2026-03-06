%% HW5_env.m
% Clear the workspace and add helper functions / API wrappers
clc
clear all
close all
addpath("Api"); % Ensure your API folder is in the path

%% Goal
% Place the target object at a random pose, solve IK to move UR5e to the target,
% then run a short stepped simulation where we compute gravity-compensating joint torques (including the applied force on z-axis of EFF frame),
% so the robot stays (approximately) stationary while logging/plotting data.

% Initialize Communication with CoppeliaSim

clientID = -1;
[ret_status, sim, clientID] = initializeComm();

if (ret_status == 0)
    % SAFETY: Always wrap simulation calls so we can stop/cleanup on error
    try
        % Use stepping mode so MATLAB controls simulation time explicitly
        sim.setStepping(true);

        %% 1) Get object handles from the CoppeliaSim scene
        % These handles are used for:
        %   - reading joint angles
        %   - applying computed joint torques
        %   - reading the wrist force sensor
        %   - querying transforms for plotting and debugging
        try
            % named in coppeliasim
            joint_IDs_ur5e{1} = getObjectReference(sim, 'shoulder_pan_joint');
            joint_IDs_ur5e{2} = getObjectReference(sim, 'shoulder_lift_joint');
            joint_IDs_ur5e{3} = getObjectReference(sim, 'elbow_joint');
            joint_IDs_ur5e{4} = getObjectReference(sim, 'wrist_1_joint');
            joint_IDs_ur5e{5} = getObjectReference(sim, 'wrist_2_joint');
            joint_IDs_ur5e{6} = getObjectReference(sim, 'wrist_3_joint');


            joint_IDs_jaco{1} = getObjectReference(sim, 'joint1');
            joint_IDs_jaco{2} = getObjectReference(sim, 'joint2');
            joint_IDs_jaco{3} = getObjectReference(sim, 'joint3');
            joint_IDs_jaco{4} = getObjectReference(sim, 'joint4');
            joint_IDs_jaco{5} = getObjectReference(sim, 'joint5');
            joint_IDs_jaco{6} = getObjectReference(sim, 'joint6');

            baseFrameID_ur5e = getObjectReference(sim, 'base_link_inertia_visual');
            endFrameID_ur5e  = getObjectReference(sim, 'feltPen_ur5e');


            baseFrameID_jaco = getObjectReference(sim, 'link1_visible');
            endFrameID_jaco  = getObjectReference(sim, 'feltPen_jaco');

            

            ObjID_ur5e = getObjectReference(sim,'target_ur5e');
            ObjID_jaco = getObjectReference(sim,'target_jaco');

        catch ME
            error('Could not find one of the objects. Error: %s', ME.message);
        end
        wall_ur5e = -2.15;
        wall_jaco = -2; %wall: -2.152

        obj_pos_ur5e = [wall_ur5e; -0.9; 1];
        obj_ori_ur5e = [0;0;0];

        %obj_pos_jaco = [wall_jaco; 1.2; 0.7];
        obj_ori_jaco = [0;0;0];

        
        y_range = [ 0.5, 1];
        z_range = [ 1,  1.3]; % (currently only using z_range(1) below)

        obj_pos_jaco = [
            wall_jaco;
            y_range(1) + rand*(y_range(2)-y_range(1));
            z_range(1) + rand*(z_range(2)-z_range(1));
        ];
        
        % sim.setObjectPosition(ObjID_ur5e, num2cell(obj_pos_ur5e));
        % sim.setObjectOrientation(ObjID_ur5e, num2cell(obj_ori_ur5e));
        % sim.setObjectPosition(ObjID_jaco, num2cell(obj_pos_jaco));
        % sim.setObjectOrientation(ObjID_jaco, num2cell(obj_ori_jaco));

        sim.setObjectPosition(ObjID_ur5e, -1, obj_pos_ur5e);
        sim.setObjectOrientation(ObjID_ur5e, -1, obj_ori_ur5e);

        sim.setObjectPosition(ObjID_jaco, -1, obj_pos_jaco);
        sim.setObjectOrientation(ObjID_jaco, -1, obj_ori_jaco);

        ur5eOrient = cell2mat(sim.getObjectOrientation(baseFrameID_ur5e, -1));
        jacoOrient = cell2mat(sim.getObjectOrientation(baseFrameID_jaco, -1));
        fprintf('UR5e base orient: %.4f %.4f %.4f\n', ur5eOrient(1), ur5eOrient(2), ur5eOrient(3));
        fprintf('Jaco base orient: %.4f %.4f %.4f\n', jacoOrient(1), jacoOrient(2), jacoOrient(3));

        
        

        %% 3) Parse URDF once (kinematics + inertial properties)
        % joints: joint tree, axes, and origins (for kinematics/Jacobian, etc.)
        % linkInertials : inertial info (mass, COM offset, inertia tensor) for gravity torques
        ur5ePath = 'ur5e.urdf';
        jacoPath = 'j2n6s300.urdf';
        joints_ur5e = parseUrdfJoints(ur5ePath);
        joints_jaco = parseUrdfJoints(jacoPath);
        joints_jaco = joints_jaco(1:end-1);  % exclude j2n6s300_joint_end_effector
        linkInertials_ur5e  = parseUrdfLinkInertials(ur5ePath);
        linkInertials_jaco  = parseUrdfLinkInertials(jacoPath);

        

        % After parsing, prepend a virtual base joint that accounts for 
        % where the Jaco actually sits in the CoppeliaSim world
        basePos    = cell2mat(sim.getObjectPosition(baseFrameID_jaco, -1));
        baseOrient = cell2mat(sim.getObjectOrientation(baseFrameID_jaco, -1));

        virtualBase.name   = 'virtual_base';
        virtualBase.type   = 'fixed';
        virtualBase.parent = '';
        virtualBase.child  = '';
        virtualBase.xyz    = basePos(:);
        virtualBase.rpy    = baseOrient(:);
        virtualBase.axis   = [0;0;1];

        % Prepend to joints array
        joints_jaco = [virtualBase, joints_jaco];

   
        %% 4) Set initial robot configuration using custom IK (before simulation)
        % For UR5e (perpendicular to wall):
        targetOrient_ur5e = [pi/2; 0; 0];

        %For Jaco (tune these values to get perpendicular):
        targetOrient_jaco = [0; pi/2; 0];  % adjust until EEF is perpendicular

        

        jointAngles_ur5e = Final_functions.getJointAngles_IK(sim, joints_ur5e, ObjID_ur5e, baseFrameID_ur5e, endFrameID_ur5e, joint_IDs_ur5e, targetOrient_ur5e);
        jointAngles_jaco = Final_functions.getJointAngles_IK(sim, joints_jaco, ObjID_jaco, baseFrameID_jaco, endFrameID_jaco, joint_IDs_jaco, targetOrient_jaco);
        

        for i=1:6
            % Torque/force control mode (MuJoCo-based dyn ctrl in Coppelia)
            sim.setObjectInt32Param(joint_IDs_ur5e{i}, sim.jointintparam_dynctrlmode, sim.jointdynctrl_force);
            sim.setObjectInt32Param(joint_IDs_jaco{i}, sim.jointintparam_dynctrlmode, sim.jointdynctrl_force);

            % Set joint state and targets consistently at the initial configuration
            sim.setJointPosition(joint_IDs_ur5e{i}, jointAngles_ur5e(i));
            sim.setJointTargetPosition(joint_IDs_ur5e{i}, jointAngles_ur5e(i));
            sim.setJointTargetVelocity(joint_IDs_ur5e{i}, 0);
            sim.setJointTargetForce(joint_IDs_ur5e{i}, 0);

            sim.setJointPosition(joint_IDs_jaco{i}, jointAngles_jaco(i));
            sim.setJointTargetPosition(joint_IDs_jaco{i}, jointAngles_jaco(i));
            sim.setJointTargetVelocity(joint_IDs_jaco{i}, 0);
            sim.setJointTargetForce(joint_IDs_jaco{i}, 0);
        end

        % Give the scene a moment to update to the teleported configuration
        pause(2)
        
        %% Setup drawing object
       
        drawingHandle_ur5e = sim.addDrawingObject(sim.drawing_lines + sim.drawing_cyclic, ...
                                     2, 0, -1, 100000, {1, 1, 1});
        penTipHandle_ur5e = getObjectReference(sim, 'tip_ur5e');

        drawingHandle_jaco = sim.addDrawingObject(sim.drawing_lines + sim.drawing_cyclic, ...
                                     2, 0, -1, 100000, {1, 1, 1});
        penTipHandle_jaco = getObjectReference(sim, 'tip_jaco');


        %Final_functions.drawLine(sim, -1, -0.5, 0.7, wall_ur5e, 50, ObjID_ur5e, joint_IDs_ur5e, joints_ur5e, baseFrameID_ur5e, endFrameID_ur5e, penTipHandle_ur5e, drawingHandle_ur5e, targetOrient_ur5e);
        %Final_functions.drawLine(sim, 0.3, 0.7, 0.3, wall_jaco, 50, ObjID_jaco, joint_IDs_jaco, joints_jaco, baseFrameID_jaco, endFrameID_jaco, penTipHandle_jaco, drawingHandle_jaco, targetOrient_jaco);
        % Circle centered at y=0.2, z=0.65, radius=0.1
        %Final_functions.drawCircle(sim, -0.9, 0.6, 0.5, wall_ur5e, 100, ObjID_ur5e, joint_IDs_ur5e, joints_ur5e, baseFrameID_ur5e, endFrameID_ur5e, penTipHandle_ur5e, drawingHandle_ur5e, targetOrient_ur5e);

        %% 5) Start simulation
        %break so drawing can be viewed
        sim.startSimulation();


    catch ME
        disp('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!');
        disp('An error occurred during execution:');
        disp(ME.message);
        disp('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!');
    end

    %% 8) Cleanup: stop sim + disconnect (always try, even if errors happened)
    if clientID > -1
        try
            sim.stopSimulation();
            pause(1)

            % Ensure the object is unparented (world parent) after sim ends
            sim.setObjectParent(ObjID, -1, true);
        catch
            disp('Could not stop simulation.');
        end
        uninitializeComm(sim, clientID);
        disp('Connection closed successfully.');
    end

else
    disp('Unable to connect to CoppeliaSim');
end

%% ---------------- Helper Functions ----------------
% The following URDF parsing helpers are copied from HW2_env.m and extended
% to also extract link inertial properties used for gravity torque computation.

function joints = parseUrdfJoints(urdfPath)
    % Parse URDF file and extract joint kinematic definitions
    if ~isfile(urdfPath)
        error('File not found: %s', urdfPath);
    end

    doc = xmlread(urdfPath);
    robot = doc.getDocumentElement();
    jointNodes = robot.getElementsByTagName('joint');
    n = jointNodes.getLength();

    joints = repmat(struct( ...
        'name', '', ...
        'type', '', ...
        'parent', '', ...
        'child', '', ...
        'xyz', [0; 0; 0], ...
        'rpy', [0; 0; 0], ...
        'axis', [0; 0; 1]), 1, 1);

    j = 0;

    for i = 1:n
        jNode = jointNodes.item(i-1);

        if isempty(char(jNode.getAttribute('type')))
            continue
        end
        if strcmp(char(jNode.getAttribute('name')), "base_link-base_fixed_joint")
            continue
        end

        j = j + 1;
        joints(j).name = char(jNode.getAttribute('name'));
        joints(j).type = char(jNode.getAttribute('type'));

        parentNodes = jNode.getElementsByTagName('parent');
        if parentNodes.getLength() > 0
            joints(j).parent = char(parentNodes.item(0).getAttribute('link'));
        end

        childNodes = jNode.getElementsByTagName('child');
        if childNodes.getLength() > 0
            joints(j).child = char(childNodes.item(0).getAttribute('link'));
        end

        originNodes = jNode.getElementsByTagName('origin');
        if originNodes.getLength() > 0
            originNode = originNodes.item(0);
            xyzStr = char(originNode.getAttribute('xyz'));
            if ~isempty(xyzStr)
                joints(j).xyz = str2vec3(xyzStr);
            end
            rpyStr = char(originNode.getAttribute('rpy'));
            if ~isempty(rpyStr)
                joints(j).rpy = str2vec3(rpyStr);
            end
        end

        axisNodes = jNode.getElementsByTagName('axis');
        if axisNodes.getLength() > 0
            axisStr = char(axisNodes.item(0).getAttribute('xyz'));
            if ~isempty(axisStr)
                joints(j).axis = str2vec3(axisStr);
            end
        end
    end
end

function vec = str2vec3(str)
    % Convert "x y z" string to a 3x1 numeric vector
    vec = sscanf(str, '%f');
    if numel(vec) < 3
        vec = [vec; zeros(3-numel(vec), 1)];
    elseif numel(vec) > 3
        vec = vec(1:3);
    end
    vec = vec(:);
end

function links = parseUrdfLinkInertials(urdfPath)
    % Parse URDF file and extract each link's inertial properties:
    %   - mass
    %   - inertial origin (xyz,rpy) relative to link frame
    %   - inertia tensor about COM (symmetric 3x3)
    if ~isfile(urdfPath)
        error('File not found: %s', urdfPath);
    end

    doc = xmlread(urdfPath);
    robot = doc.getDocumentElement();
    linkNodes = robot.getElementsByTagName('link');
    n = linkNodes.getLength();

    links = repmat(struct( ...
        'name', '', ...
        'hasInertial', false, ...
        'mass', 0.0, ...
        'xyz', [0;0;0], ...
        'rpy', [0;0;0], ...
        'I', zeros(3,3) ...
        ), n, 1);

    for i = 1:n
        linkNode = linkNodes.item(i-1);
        links(i).name = char(linkNode.getAttribute('name'));

        inertialNodes = linkNode.getElementsByTagName('inertial');
        if inertialNodes.getLength() == 0
            continue;
        end

        inertialNode = inertialNodes.item(0);
        links(i).hasInertial = true;

        massNodes = inertialNode.getElementsByTagName('mass');
        if massNodes.getLength() > 0
            massStr = char(massNodes.item(0).getAttribute('value'));
            if ~isempty(massStr)
                links(i).mass = str2double(massStr);
            end
        end

        originNodes = inertialNode.getElementsByTagName('origin');
        if originNodes.getLength() > 0
            originNode = originNodes.item(0);

            xyzStr = char(originNode.getAttribute('xyz'));
            if ~isempty(xyzStr)
                links(i).xyz = str2vec3(xyzStr);
            end

            rpyStr = char(originNode.getAttribute('rpy'));
            if ~isempty(rpyStr)
                links(i).rpy = str2vec3(rpyStr);
            end
        end

        inertiaNodes = inertialNode.getElementsByTagName('inertia');
        if inertiaNodes.getLength() > 0
            inertiaNode = inertiaNodes.item(0);

            ixx = str2double(char(inertiaNode.getAttribute('ixx')));
            ixy = str2double(char(inertiaNode.getAttribute('ixy')));
            ixz = str2double(char(inertiaNode.getAttribute('ixz')));
            iyy = str2double(char(inertiaNode.getAttribute('iyy')));
            iyz = str2double(char(inertiaNode.getAttribute('iyz')));
            izz = str2double(char(inertiaNode.getAttribute('izz')));

            links(i).I = [ ixx, ixy, ixz;
                           ixy, iyy, iyz;
                           ixz, iyz, izz ];
        end
    end
end

function r = randVal_in_range(a, b)
    % Uniform random number in [a, b]
    r = a + (b-a) * rand();
end


