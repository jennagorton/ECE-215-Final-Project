classdef Final_functions
    % HW5_ANSWER
    % Static methods used in HW5 for:
    %   (1) solving inverse kinematics (IK) to align the UR5e end-effector to a target pose, and
    %   (2) computing joint torques for static equilibrium under gravity and an external end-effector force.


    methods (Static)

        
        function jointAngles = getJointAngles_IK(sim, joints, targetObjID, robotBaseID, endFrameID, joint_IDs, targetOrient)
            % getJointAngles_IK
            % Inverse kinematics (IK) solver that computes jointAngles so the
            % end-effector frame matches the target object's pose (position + orientation).

            % Inputs
            %   sim         : CoppeliaSim remote API object
            %   joints      : URDF joint struct array (kinematic chain)
            %   targetObjID : CoppeliaSim handle of the target object (desired pose)
            %   robotBaseID : CoppeliaSim handle of the robot base reference frame
            %   endFrameID  : end-effector handle 
            %
            % Output
            %   jointAngles : 6x1 joint angles [rad], wrapped to [-pi, pi]
            
            jointAngles = zeros(6,1);

            %============== Your code here=================
            maxIter = 50;           % this seems big enough
            tol = 0.001;              % tested different values
            
            % initalize joint angles as the intial position instead of zero matrix 
            for j = 1:6
                jointAngles(j) = sim.getJointPosition(joint_IDs{j});
            end

           
            
            % only compuyte 1000 iterations
            for k = 1:maxIter 
                
                % get target position and orientation
                targetPos = cell2mat(sim.getObjectPosition(targetObjID, -1));
                %targetOrient = cell2mat(sim.getObjectOrientation(targetObjID, -1));
                R_target = eul2rotm(targetOrient(:)', 'ZYX');

                % Current EEF orientation
                curPos   = cell2mat(sim.getObjectPosition(endFrameID, -1));
                curOrient = cell2mat(sim.getObjectOrientation(endFrameID, -1));
                R_cur    = eul2rotm(curOrient(:)', 'ZYX');

                % Position error
                posErr = targetPos(:) - curPos(:);

                % Orientation error (now always driving toward fixedOrient)
                R_err     = R_target * R_cur';
                orientErr = 0.5 * [R_err(3,2)-R_err(2,3); R_err(1,3)-R_err(3,1); R_err(2,1)-R_err(1,2)];

                err = [posErr; orientErr];  % only care about position
                %disp(err);

                errNorm = norm(err);
            
                % check for convergence
                if norm(err) < tol
                    break;
                end

                % get jacobian
                J = getJacobian(jointAngles, joints);

            
                % newton raphson equation
                lambda = 0.1;  % damping factor
                alpha = min(0.1, errNorm); % when error really small, make alpha really small
                deltaTheta = alpha * J' * ((J*J' + lambda^2 * eye(6)) \ err);


                % fprintf('robot: %d\n', joint_IDs{1});
                % fprintf('deltaTheta: %.4f %.4f %.4f %.4f %.4f %.4f\n', deltaTheta);
                % fprintf('jointAngles: %.4f %.4f %.4f %.4f %.4f %.4f\n', jointAngles);
                % fprintf('J linear rows:\n');
                % disp(J(1:3,:));


                % update joint angles
                jointAngles = jointAngles + deltaTheta;
                jointAngles = wrapToPi(jointAngles);
                
                % move actual robot to new joint angles
               for j = 1:length(joint_IDs)
                    sim.setJointPosition(joint_IDs{j}, jointAngles(j));
               end
            end
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % draw line function
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        function drawLine(sim, y_start, y_end, z_pos, wall, n_steps, ObjID, joint_IDs, joints, baseFrameID, endFrameID, penTipHandle, drawingHandle, targetOrient)
            y_vals = linspace(y_start, y_end, n_steps);
            prevPos = [];

            for step = 1:n_steps
                % Set target position
                obj_pos = [wall; y_vals(step); z_pos];
                sim.setObjectPosition(ObjID, num2cell(obj_pos));
                sim.setObjectOrientation(ObjID, num2cell([0; 0; 0]));

                % Solve IK
                jointAngles = Final_functions.getJointAngles_IK(sim, joints, ObjID, baseFrameID, endFrameID, joint_IDs, targetOrient);

                % Apply joint targets
                for i = 1:6
                    sim.setObjectInt32Param(joint_IDs{i}, sim.jointintparam_dynctrlmode, sim.jointdynctrl_force);
                    sim.setJointPosition(joint_IDs{i},       jointAngles(i));
                    sim.setJointTargetPosition(joint_IDs{i}, jointAngles(i));
                    sim.setJointTargetVelocity(joint_IDs{i}, 0);
                    sim.setJointTargetForce(joint_IDs{i},    0);
                end

                % Step simulation and record drawing
                sim.step();
                currPos = sim.getObjectPosition(penTipHandle, -1);
                if ~isempty(prevPos)
                    sim.addDrawingObjectItem(drawingHandle, [prevPos, currPos]);
                end
                prevPos = currPos;
            end
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Draw circle function
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        function drawCircle(sim, cy, cz, radius, wall, n_steps, ObjID, joint_IDs, joints, baseFrameID, endFrameID, penTipHandle, drawingHandle, targetOrient)
            % cy, cz = center of circle in y and z
            % radius = circle radius
            % n_steps = number of points (more = smoother)
    
            angles = linspace(0, 2*pi, n_steps);
            prevPos = [];

            for step = 1:n_steps
                % Parametric circle in y-z plane
                y = cy + radius * cos(angles(step));
                z = cz + radius * sin(angles(step));

                % Set target position
                sim.setObjectPosition(ObjID, num2cell([wall; y; z]));
                sim.setObjectOrientation(ObjID, num2cell([0; 0; 0]));

                % Solve IK
                jointAngles = Final_functions.getJointAngles_IK(sim, joints, ObjID, baseFrameID, endFrameID, joint_IDs, targetOrient);

                % Apply joint targets
                for i = 1:6
                    sim.setObjectInt32Param(joint_IDs{i}, sim.jointintparam_dynctrlmode, sim.jointdynctrl_force);
                    sim.setJointPosition(joint_IDs{i},       jointAngles(i));
                    sim.setJointTargetPosition(joint_IDs{i}, jointAngles(i));
                    sim.setJointTargetVelocity(joint_IDs{i}, 0);
                    sim.setJointTargetForce(joint_IDs{i},    0);
                end

                % Step simulation and record drawing
                sim.step();
                currPos = sim.getObjectPosition(penTipHandle, -1);
                if ~isempty(prevPos)
                    sim.addDrawingObjectItem(drawingHandle, [prevPos, currPos]);
                end
                prevPos = currPos;
            end

            % Close the circle by connecting last point back to first
            sim.setObjectPosition(ObjID, num2cell([wall; cy + radius; cz]));
            sim.setObjectOrientation(ObjID, num2cell([0; 0; 0]));
            jointAngles = Final_functions.getJointAngles_IK(sim, joints, ObjID, baseFrameID, endFrameID, joint_IDs, targetOrient);
            for i = 1:6
                sim.setJointPosition(joint_IDs{i},       jointAngles(i));
                sim.setJointTargetPosition(joint_IDs{i}, jointAngles(i));
            end
            sim.step();
            closingPos = sim.getObjectPosition(penTipHandle, -1);
            sim.addDrawingObjectItem(drawingHandle, [prevPos, closingPos]);
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Jacobian Function from hw solutions
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        function Jacobian_mat = getJacobian1(jointAngles, joints, sim, baseFrameID)

            
        
            jointAngles = jointAngles(:); % Ensure column vector
            RobotDOF = length(jointAngles);
            Jacobian_mat = zeros(6, RobotDOF);


            for activeJointID = 1:RobotDOF
                w_i_i = zeros(3,1);
                v_i_i = zeros(3,1);

                % Initialize with base transform instead of eye(4)
                basePos    = cell2mat(sim.getObjectPosition(baseFrameID, -1));
                baseOrient = cell2mat(sim.getObjectOrientation(baseFrameID, -1));
                R_base = eul2rotm(baseOrient(:)', 'XYZ');
                T_world_to_target = eye(4);
                T_world_to_target(1:3,1:3) = R_base;
                T_world_to_target(1:3,4)   = basePos(:);

                qIdx = 1;

                for k = 1:numel(joints)
                    J = joints(k);
                
                    % static transform, urdf origin
                    R_static = eul2rotm(J.rpy', 'XYZ');
                    p_static = J.xyz;

                    theta_dot_ip1 = 0;
                    jointAxis = J.axis;

                    % joint transform

                    R_joint = eye(3);

                    if strcmp(J.type, 'revolute') || strcmp(J.type, 'continuous')
                        q = jointAngles(qIdx);

                        R_joint = axang2rotm([jointAxis', q]);

                        if qIdx == activeJointID
                            theta_dot_ip1 = 1.0;
                        end
                        qIdx = qIdx + 1;
                    end
                
                    % combine transforms to get T(i->i+1)
                    R_prev_curr = R_static * R_joint;
                    p_prev_curr = p_static;

                    % velocity propagation
                    R_curr_prev = R_prev_curr.';

                    if isempty(jointAxis)
                        jointAxis = [0;0;1];
                    end

                    % w_{i+1} = R_{i+1,i} * w_i + theta_dot * z_{i+1}
                    w_ip1_ip1 = R_curr_prev * w_i_i + theta_dot_ip1 * jointAxis;
                    % v_{i+1} = R_{i+1,i} * (v_i + w_i x p_{i, i+1})
                    v_ip1_ip1 = R_curr_prev * (v_i_i + cross(w_i_i, p_prev_curr));

                    w_i_i = w_ip1_ip1;
                    v_i_i = v_ip1_ip1;

                    T_this = eye(4);
                    T_this(1:3, 1:3) = R_prev_curr;
                    T_this(1:3, 4) = p_prev_curr;
                    T_world_to_target = T_world_to_target * T_this;

                end

                % transform velocities back to world frame
                R_0_N = T_world_to_target(1:3, 1:3);

                v_base = R_0_N * v_i_i;
                w_base = R_0_N * w_i_i;

                Jacobian_mat(:, activeJointID) = [v_base; w_base];
          

            end


        end
    
    end
end

        


%% HELPER FUNCTION

function Jacobian_mat = getJacobian(jointAngles, joints)

    jointAngles = jointAngles(:);
    RobotDOF = length(jointAngles);
    Jacobian_mat = zeros(6, RobotDOF);

    Rx = @(r)[1 0 0; 0 cos(r) -sin(r); 0 sin(r) cos(r)];
    Ry = @(p)[cos(p) 0 sin(p); 0 1 0; -sin(p) 0 cos(p)];
    Rz = @(y)[cos(y) -sin(y) 0; sin(y) cos(y) 0; 0 0 1];

    p = zeros(3, RobotDOF);
    z = zeros(3, RobotDOF);

    T = eye(4);
    qIdx = 0;

    for i = 1:length(joints)
        xyz = joints(i).xyz;
        rpy = joints(i).rpy;
        R_origin = Rz(rpy(3)) * Ry(rpy(2)) * Rx(rpy(1));
        T_origin = eye(4);
        T_origin(1:3,1:3) = R_origin;
        T_origin(1:3,4) = xyz;
        T = T * T_origin;

        if strcmp(joints(i).type,'revolute') || strcmp(joints(i).type,'continuous')
            qIdx = qIdx + 1;

            % joint axis and position in world frame
            z(:,qIdx) = T(1:3,1:3) * joints(i).axis(:);
            p(:,qIdx) = T(1:3,4);

            % apply joint rotation using Rodrigues
            ax = joints(i).axis(:);
            theta = jointAngles(qIdx);
            K = [0,-ax(3),ax(2); ax(3),0,-ax(1); -ax(2),ax(1),0];
            R_joint = eye(3) + sin(theta)*K + (1-cos(theta))*(K*K);
            T_joint = eye(4);
            T_joint(1:3,1:3) = R_joint;
            T = T * T_joint;
        end
    end

    p_ee = T(1:3,4);

    % Build Jacobian
    for i = 1:RobotDOF
        Jv = cross(z(:,i), p_ee - p(:,i));
        Jw = z(:,i);
        Jacobian_mat(:,i) = [Jv; Jw];
    end
end


     