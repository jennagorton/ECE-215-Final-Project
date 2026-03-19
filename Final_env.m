%% HW5_env.m
% Clear the workspace and add helper functions / API wrappers
clc
clear all
close all
addpath("Api"); % Ensure your API folder is in the path

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
            %% Getting object reference for each arm component
            joint_IDs_ur5e_a{1} = getObjectReference(sim, 'shoulder_pan_joint');
            joint_IDs_ur5e_a{2} = getObjectReference(sim, 'shoulder_lift_joint');
            joint_IDs_ur5e_a{3} = getObjectReference(sim, 'elbow_joint');
            joint_IDs_ur5e_a{4} = getObjectReference(sim, 'wrist_1_joint');
            joint_IDs_ur5e_a{5} = getObjectReference(sim, 'wrist_2_joint');
            joint_IDs_ur5e_a{6} = getObjectReference(sim, 'wrist_3_joint');

            joint_IDs_jaco_a{1} = getObjectReference(sim, 'joint1');
            joint_IDs_jaco_a{2} = getObjectReference(sim, 'joint2');
            joint_IDs_jaco_a{3} = getObjectReference(sim, 'joint3');
            joint_IDs_jaco_a{4} = getObjectReference(sim, 'joint4');
            joint_IDs_jaco_a{5} = getObjectReference(sim, 'joint5');
            joint_IDs_jaco_a{6} = getObjectReference(sim, 'joint6');

            baseFrameID_ur5e_a = getObjectReference(sim, 'base_link_inertia_visual');
            endFrameID_ur5e_a  = getObjectReference(sim, 'tip_ur5e');
            disp(endFrameID_ur5e_a);

            baseFrameID_jaco_a = getObjectReference(sim, 'link1_visible');
            endFrameID_jaco_a  = getObjectReference(sim, 'tip_jaco');
            disp(endFrameID_jaco_a);

            joint_IDs_ur5e_b{1} = getObjectReference(sim, 'shoulder_pan_joint_b');
            joint_IDs_ur5e_b{2} = getObjectReference(sim, 'shoulder_lift_joint_b');
            joint_IDs_ur5e_b{3} = getObjectReference(sim, 'elbow_joint_b');
            joint_IDs_ur5e_b{4} = getObjectReference(sim, 'wrist_1_joint_b');
            joint_IDs_ur5e_b{5} = getObjectReference(sim, 'wrist_2_joint_b');
            joint_IDs_ur5e_b{6} = getObjectReference(sim, 'wrist_3_joint_b');

            joint_IDs_jaco_b{1} = getObjectReference(sim, 'joint1_b');
            joint_IDs_jaco_b{2} = getObjectReference(sim, 'joint2_b');
            joint_IDs_jaco_b{3} = getObjectReference(sim, 'joint3_b');
            joint_IDs_jaco_b{4} = getObjectReference(sim, 'joint4_b');
            joint_IDs_jaco_b{5} = getObjectReference(sim, 'joint5_b');
            joint_IDs_jaco_b{6} = getObjectReference(sim, 'joint6_b');

            baseFrameID_ur5e_b = getObjectReference(sim, 'base_link_inertia_visual_b');
            endFrameID_ur5e_b  = getObjectReference(sim, 'tip_ur5e_b');
            disp(endFrameID_ur5e_b);

            baseFrameID_jaco_b = getObjectReference(sim, 'link1_visible_b');
            endFrameID_jaco_b  = getObjectReference(sim, 'tip_jaco_b');
            disp(endFrameID_jaco_b);

            ObjID_ur5e_a = getObjectReference(sim,'target_ur5e');
            ObjID_jaco_a = getObjectReference(sim,'target_jaco');
            ObjID_ur5e_b = getObjectReference(sim,'target_ur5e1');
            ObjID_jaco_b = getObjectReference(sim,'target_jaco1');

        catch ME
            error('Could not find one of the objects. Error: %s', ME.message);
        end
        % hardcode wall pose
        wall_ur5e = -2.2;
        wall_jaco = -2.18; 

        obj_pos_ur5e = [wall_ur5e; -0.9; 1];
        obj_ori_ur5e = [0;0;0];

        obj_ori_jaco = [0;0;0];


        y_range = [ 0.5, 1];
        z_range = [ 1,  1.3];

        obj_pos_jaco = [
            wall_jaco;
            y_range(1) + rand*(y_range(2)-y_range(1));
            z_range(1) + rand*(z_range(2)-z_range(1));
            ];

        floor_ur5e = 0.21;
        floor_jaco = 0.21;

        obj_pos_ur5e1 = [0.6; -1; floor_ur5e];
        obj_pos_jaco1 = [0.8; 0.8; floor_jaco];

        % set all initial object positions
        sim.setObjectPosition(ObjID_ur5e_a, -1, obj_pos_ur5e);
        sim.setObjectOrientation(ObjID_ur5e_a, -1, obj_ori_ur5e);

        sim.setObjectPosition(ObjID_jaco_a, -1, obj_pos_jaco);
        sim.setObjectOrientation(ObjID_jaco_a, -1, obj_ori_jaco);

        sim.setObjectPosition(ObjID_ur5e_b, -1, obj_pos_ur5e1);
        sim.setObjectOrientation(ObjID_ur5e_b, -1, obj_ori_ur5e);

        sim.setObjectPosition(ObjID_jaco_b, -1, obj_pos_jaco1);
        sim.setObjectOrientation(ObjID_jaco_b, -1, obj_ori_jaco);


        %% 3) Parse URDF once (kinematics + inertial properties)
        % joints: joint tree, axes, and origins (for kinematics/Jacobian, etc.)
        % linkInertials : inertial info (mass, COM offset, inertia tensor) for gravity torques
        ur5ePath_a = 'ur5e.urdf';
        jacoPath_a = 'j2n6s300.urdf';
        joints_ur5e_a = parseUrdfJoints(ur5ePath_a);
        joints_jaco_a = parseUrdfJoints(jacoPath_a);
        joints_jaco_a = joints_jaco_a(1:end-1);  % exclude j2n6s300_joint_end_effector
        linkInertials_ur5e_a  = parseUrdfLinkInertials(ur5ePath_a);
        linkInertials_jaco_a  = parseUrdfLinkInertials(jacoPath_a);

        ur5ePath_b = 'ur5e.urdf';
        jacoPath_b = 'j2n6s300.urdf';
        joints_ur5e_b = parseUrdfJoints(ur5ePath_b);
        joints_jaco_b = parseUrdfJoints(jacoPath_b);
        joints_jaco_b = joints_jaco_b(1:end-1);  % exclude j2n6s300_joint_end_effector
        linkInertials_ur5e_b  = parseUrdfLinkInertials(ur5ePath_b);
        linkInertials_jaco_b  = parseUrdfLinkInertials(jacoPath_b);

        % After parsing, prepend a virtual base joint that accounts for
        % where the Jaco actually sits in the CoppeliaSim world
        basePos    = cell2mat(sim.getObjectPosition(baseFrameID_jaco_a, -1));
        baseOrient = cell2mat(sim.getObjectOrientation(baseFrameID_jaco_a, -1));

        virtualBase.name   = 'virtual_base';
        virtualBase.type   = 'fixed';
        virtualBase.parent = '';
        virtualBase.child  = '';
        virtualBase.xyz    = basePos(:);
        virtualBase.rpy    = baseOrient(:);
        virtualBase.axis   = [0;0;1];

        % Prepend to joints array
        joints_jaco_a = [virtualBase, joints_jaco_a];

        basePos_b    = cell2mat(sim.getObjectPosition(baseFrameID_jaco_b, -1));
        baseOrient_b = cell2mat(sim.getObjectOrientation(baseFrameID_jaco_b, -1));

        virtualBase_b.name   = 'virtual_base';
        virtualBase_b.type   = 'fixed';
        virtualBase_b.parent = '';
        virtualBase_b.child  = '';
        virtualBase_b.xyz    = basePos_b(:);
        virtualBase_b.rpy    = baseOrient_b(:);
        virtualBase_b.axis   = [0;0;1];

        % Prepend to joints array
        joints_jaco_b = [virtualBase_b, joints_jaco_b];


        %% 4) Set initial robot configuration using custom IK (before simulation)
        % For UR5e (perpendicular to wall):
        targetOrient_ur5e_a = [0; 0; 0];
        targetOrient_ur5e_b = [0; pi/2; 0];

        %For Jaco (tune these values to get perpendicular):
        targetOrient_jaco_a = [0; 0; 0];  % adjust until EEF is perpendicular
        targetOrient_jaco_b = [0; 0; 0];
        
        % initialize floor drawing to elbow up
        Final_functions.setElbowUp(sim, joint_IDs_ur5e_b, joint_IDs_jaco_b);
        
        %% read inital angles of all arms
        jointAngles_jaco_b = zeros(6, 1);
        for i = 1:6
            jointAngles_jaco_b(i) = sim.getJointPosition(joint_IDs_jaco_b{i});
        end
        disp('Current jaco_b joint angles (rad):');
        disp(jointAngles_jaco_b);

        jointAngles_jaco_a = zeros(6, 1);
        for i = 1:6
            jointAngles_jaco_a(i) = sim.getJointPosition(joint_IDs_jaco_a{i});
        end
        disp('Current jaco_a joint angles (rad):');
        disp(jointAngles_jaco_a);

        jointAngles_ur5e_b = zeros(6, 1);
        for i = 1:6
            jointAngles_ur5e_b(i) = sim.getJointPosition(joint_IDs_ur5e_b{i});
        end
        disp('Current ur5e_b joint angles (rad):');
        disp(jointAngles_ur5e_b);

        jointAngles_ur5e_a = zeros(6, 1);
        for i = 1:6
            jointAngles_ur5e_a(i) = sim.getJointPosition(joint_IDs_ur5e_a{i});
        end
        disp('Current ur5e_a joint angles (rad):');
        disp(jointAngles_ur5e_a);

        %% Move robot to initial target pose
        % jointAngles_ur5e_a = Final_functions.getJointAngles_IK(sim, joints_ur5e_a, ObjID_ur5e_a, baseFrameID_ur5e_a, endFrameID_ur5e_a, joint_IDs_ur5e_a, targetOrient_ur5e_a);
        % jointAngles_jaco_a = Final_functions.getJointAngles_IK(sim, joints_jaco_a, ObjID_jaco_a, baseFrameID_jaco_a, endFrameID_jaco_a, joint_IDs_jaco_a, targetOrient_jaco_a);
        % 
        % 
        % jointAngles_ur5e_b = Final_functions.getJointAngles_IK(sim, joints_ur5e_b, ObjID_ur5e_b, baseFrameID_ur5e_b, endFrameID_ur5e_b, joint_IDs_ur5e_b, targetOrient_ur5e_b);
        % jointAngles_jaco_b = Final_functions.getJointAngles_IK(sim, joints_jaco_b, ObjID_jaco_b, baseFrameID_jaco_b, endFrameID_jaco_b, joint_IDs_jaco_b, targetOrient_jaco_b);
             

        for i=1:6
            % Torque/force control mode (MuJoCo-based dyn ctrl in Coppelia)
            sim.setObjectInt32Param(joint_IDs_ur5e_a{i}, sim.jointintparam_dynctrlmode, sim.jointdynctrl_force);
            sim.setObjectInt32Param(joint_IDs_jaco_a{i}, sim.jointintparam_dynctrlmode, sim.jointdynctrl_force);
            sim.setObjectInt32Param(joint_IDs_ur5e_b{i}, sim.jointintparam_dynctrlmode, sim.jointdynctrl_force);
            sim.setObjectInt32Param(joint_IDs_jaco_b{i}, sim.jointintparam_dynctrlmode, sim.jointdynctrl_force);


            % Set joint state and targets consistently at the initial configuration
            sim.setJointPosition(joint_IDs_ur5e_a{i}, jointAngles_ur5e_a(i));
            sim.setJointTargetPosition(joint_IDs_ur5e_a{i}, jointAngles_ur5e_a(i));
            sim.setJointTargetVelocity(joint_IDs_ur5e_a{i}, 0);
            sim.setJointTargetForce(joint_IDs_ur5e_a{i}, 0);

            sim.setJointPosition(joint_IDs_jaco_a{i}, jointAngles_jaco_a(i));
            sim.setJointTargetPosition(joint_IDs_jaco_a{i}, jointAngles_jaco_a(i));
            sim.setJointTargetVelocity(joint_IDs_jaco_a{i}, 0);
            sim.setJointTargetForce(joint_IDs_jaco_a{i}, 0);

            sim.setJointPosition(joint_IDs_ur5e_b{i}, jointAngles_ur5e_b(i));
            sim.setJointTargetPosition(joint_IDs_ur5e_b{i}, jointAngles_ur5e_b(i));
            sim.setJointTargetVelocity(joint_IDs_ur5e_b{i}, 0);
            sim.setJointTargetForce(joint_IDs_ur5e_b{i}, 0);

            sim.setJointPosition(joint_IDs_jaco_b{i}, jointAngles_jaco_b(i));
            sim.setJointTargetPosition(joint_IDs_jaco_b{i}, jointAngles_jaco_b(i));
            sim.setJointTargetVelocity(joint_IDs_jaco_b{i}, 0);
            sim.setJointTargetForce(joint_IDs_jaco_b{i}, 0);
        end

        % Give the scene a moment to update to the teleported configuration
        pause(2)

        %% Setup drawing object

        drawingHandle_ur5e_a = sim.addDrawingObject(sim.drawing_lines + sim.drawing_cyclic, ...
            2, 0, -1, 100000, {1, 1, 1});
        penTipHandle_ur5e_a = getObjectReference(sim, 'tip_ur5e');

        drawingHandle_jaco_a = sim.addDrawingObject(sim.drawing_lines + sim.drawing_cyclic, ...
            2, 0, -1, 100000, {1, 1, 1});
        penTipHandle_jaco_a = getObjectReference(sim, 'tip_jaco');

        drawingHandle_ur5e_b = sim.addDrawingObject(sim.drawing_lines + sim.drawing_cyclic, ...
            2, 0, -1, 100000, {1, 1, 1});
        penTipHandle_ur5e_b = getObjectReference(sim, 'tip_ur5e_b');

        drawingHandle_jaco_b = sim.addDrawingObject(sim.drawing_lines + sim.drawing_cyclic, ...
            2, 0, -1, 100000, {1, 1, 1});
        penTipHandle_jaco_b = getObjectReference(sim, 'tip_jaco_b');

        
        %% Setup torque logging
        F_magnitude = 5.0;          % Newtons into wall, tune this
        n_steps = 100;       % must match what you pass to drawCircle

        % Pre-allocate logs (rows = steps, cols = 6 joints)
        torque_log_ur5e    = zeros(n_steps, 6);
        tau_force_log_ur5e = zeros(n_steps, 6);
        time_log_ur5e      = zeros(n_steps, 1);

        torque_log_jaco    = zeros(n_steps, 6);
        tau_force_log_jaco = zeros(n_steps, 6);
        time_log_jaco      = zeros(n_steps, 1);

        %% Uncomment which trajectory to be drawn
        
        % [torque_log_ur5e, tau_force_log_ur5e, time_log_ur5e] = ...
        %     Final_functions.drawCircle(sim, -0.9, 0.6, 0.4, wall_ur5e, n_steps, ...
        %     ObjID_ur5e_a, joint_IDs_ur5e_a, joints_ur5e_a, baseFrameID_ur5e_a, ...
        %     endFrameID_ur5e_a, penTipHandle_ur5e_a, drawingHandle_ur5e_a, ...
        %     targetOrient_ur5e_a, F_magnitude);

        % [torque_log_jaco, tau_force_log_jaco, time_log_jaco] = ...
        %     Final_functions.drawCircle(sim, 0.7, 0.9, 0.2, wall_jaco, n_steps, ...
        %     ObjID_jaco_a, joint_IDs_jaco_a, joints_jaco_a, baseFrameID_jaco_a, ...
        %     endFrameID_jaco_a, penTipHandle_jaco_a, drawingHandle_jaco_a, ...
        %     targetOrient_jaco_a, F_magnitude);

        % quinic traj
        % obj_pos_ur5e = [wall_ur5e; -1.2; 1.3];
        % sim.setObjectPosition(ObjID_ur5e_a, -1, obj_pos_ur5e);
        % 
        % [torque_log_ur5e, tau_force_log_ur5e, time_log_ur5e] = ...
        %     Final_functions.quinticTraj(sim, joints_ur5e_a, joint_IDs_ur5e_a, baseFrameID_ur5e_a, endFrameID_ur5e_a, ...
        %     penTipHandle_ur5e_a, drawingHandle_ur5e_a, targetOrient_ur5e_a, F_magnitude, ...
        %     [], 5, 100, ObjID_ur5e_a, true);

        % [torque_log_jaco, tau_force_log_jaco, time_log_jaco] = ...
        %     Final_functions.quinticTraj(sim, joints_jaco_b, joint_IDs_jaco_b, baseFrameID_jaco_b, endFrameID_jaco_b, ...
        %     penTipHandle_jaco_b, drawingHandle_jaco_b, targetOrient_jaco_b, F_magnitude, ...
        %     [], 5, 100, ObjID_jaco_b, true);

        
        % [torque_log_ur5e, tau_force_log_ur5e, time_log_ur5e] = ...
        %     Final_functions.drawCircleFloor(sim, 0.5, -0.8, 0.07, floor_ur5e, n_steps, ...
        %     ObjID_ur5e_b, joint_IDs_ur5e_b, joints_ur5e_b, baseFrameID_ur5e_b, ...
        %     endFrameID_ur5e_b, penTipHandle_ur5e_b, drawingHandle_ur5e_b, ...
        %     targetOrient_ur5e_b, F_magnitude);
        
        [torque_log_ur5e, tau_force_log_ur5e, time_log_ur5e] = ...
            Final_functions.drawCircleFloor(sim, 0.5, -0.8, 0.15, floor_ur5e, n_steps, ...
            ObjID_ur5e_b, joint_IDs_ur5e_b, joints_ur5e_b, baseFrameID_ur5e_b, ...
            endFrameID_ur5e_b, penTipHandle_ur5e_b, drawingHandle_ur5e_b, ...
            targetOrient_ur5e_b, F_magnitude);
        % 
        % [torque_log_jaco, tau_force_log_jaco, time_log_jaco] = ...
        %     Final_functions.drawCircleFloor(sim, 0.7, 0.6, 0.07, floor_jaco, n_steps, ...
        %     ObjID_jaco_b, joint_IDs_jaco_b, joints_jaco_b, baseFrameID_jaco_b, ...
        %     endFrameID_jaco_b, penTipHandle_jaco_b, drawingHandle_jaco_b, ...
        %     targetOrient_jaco_b, F_magnitude);
        % 
        % [torque_log_jaco, tau_force_log_jaco, time_log_jaco] = ...
        %     Final_functions.drawCircleFloor(sim, 0.7, 0.6, 0.15, floor_jaco, n_steps, ...
        %     ObjID_jaco_b, joint_IDs_jaco_b, joints_jaco_b, baseFrameID_jaco_b, ...
        %     endFrameID_jaco_b, penTipHandle_jaco_b, drawingHandle_jaco_b, ...
        %     targetOrient_jaco_b, F_magnitude);


        % [torque_log, tau_force_log, time_log] = Final_functions.drawTriangle(...
        %     sim, -0.9, 0.6, 0.3, wall_ur5e, n_steps, ObjID_ur5e_a, joint_IDs_ur5e_a, joints_ur5e_a, ...
        %     baseFrameID_ur5e_a, endFrameID_ur5e_a, penTipHandle_ur5e_a, drawingHandle_ur5e_a, targetOrient_ur5e_a, F_magnitude);

        % [torque_log, tau_force_log, time_log] = Final_functions.drawTriangle(...
        %     sim, 0.7, 0.7, 0.3, wall_jaco, n_steps, ObjID_jaco_a, joint_IDs_jaco_a, joints_jaco_a, ...
        %     baseFrameID_jaco_a, endFrameID_jaco_a, penTipHandle_jaco_a, drawingHandle_jaco_a, targetOrient_jaco_a, F_magnitude);


        %% 5) Start simulation
        %break so drawing can be viewed
        pause(10);
        Xsim.startSimulation();

    catch ME
        disp('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!');
        disp('An error occurred during execution:');
        disp(ME.message);
        disp('Stack trace:');
        for k = 1:length(ME.stack)
            fprintf('  File: %s, Line: %d, Function: %s\n', ...
                ME.stack(k).file, ME.stack(k).line, ME.stack(k).name);
        end
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



