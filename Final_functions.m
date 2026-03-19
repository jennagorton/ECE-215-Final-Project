classdef Final_functions
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
            maxIter = 150;           % this seems big enough
            tol = 0.05;              % tested different values

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

                

                % choose damping based on manipulability
                manipulability = sqrt(max(0, det(J * J')));
                if manipulability < 0.01 && (endFrameID == 71 || endFrameID == 129)
                %     fprintf('Singular config: q = [%.2f %.2f %.2f %.2f %.2f %.2f] manip=%.6f\n', ...
                % jointAngles(1), jointAngles(2), jointAngles(3), ...
                % jointAngles(4), jointAngles(5), jointAngles(6), manipulability);
                    lambda = 0.5;   % heavy damping near singularity
                    alpha  = 0.02;  % tiny steps near singularity
                else
                    lambda = 0.1;
                    alpha  = min(0.1, errNorm);
                end


                
                deltaTheta = alpha * J' * ((J*J' + lambda^2 * eye(6)) \ err);
                % update joint angles
                jointAngles = jointAngles + deltaTheta;

                % Check singularity on wrapped value, but don't change jointAngles
                q_wrapped = wrapToPi(jointAngles);

                if endFrameID == 71 || endFrameID == 129
                    % E: theta4 near 0 or pi
                    if abs(sin(q_wrapped(4))) < 0.2
                        jointAngles(4) = jointAngles(4) + (sign(q_wrapped(4)) * asin(0.2) - q_wrapped(4));
                    end
                    % B1: theta5 near 0 or pi
                    if abs(sin(q_wrapped(5))) < 0.05
                        jointAngles(5) = jointAngles(5) + (sign(q_wrapped(5)) * asin(0.05) - q_wrapped(5));
                        if abs(q_wrapped(5)) < 0.01; jointAngles(5) = jointAngles(5) + 0.05; end
                    end
                    % C2: theta3 near 0
                    if abs(sin(q_wrapped(3))) < 0.05
                        jointAngles(3) = jointAngles(3) + (sign(q_wrapped(3)) * asin(0.05) - q_wrapped(3));
                        if abs(q_wrapped(3)) < 0.01; jointAngles(3) = jointAngles(3) + 0.05; end
                    end
                    if abs(cos(q_wrapped(5))) < 0.05
                        jointAngles(5) = jointAngles(5) + (sign(q_wrapped(5)) * (pi/2 - asin(0.05)) - q_wrapped(5));
                    end
                end


                if endFrameID == 71 || endFrameID == 129
                    sing_info = Final_functions.checkJacoSingularity(jointAngles, 0.15);
                    if sing_info.isSingular == 1
                        %disp(sing_info);
                    end
                end

               
                % move actual robot to new joint angles
                for j = 1:length(joint_IDs)
                    sim.setJointPosition(joint_IDs{j}, jointAngles(j));
                end

            end
            
        end




        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %  Singularity detection using Bonev et al. algebraic conditions
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        function info = checkJacoSingularity(q, thr)
            % Algebraic singularity conditions from Bonev et al. 2024.
            % q  : 6×1 joint angles [rad]
            % thr: nearness threshold (scalar)
            %
            % Returns struct with fields:
            %   isSingular (bool), type (string description)

            info.isSingular = false;
            info.type       = '';
            types           = {};

            % Kinova Link 6 DH parameters [meters]
            d3 = -0.022447;
            d4 = -0.374910;
            d5 = -0.139879;
            a2 =  0.485000;
            a5 = -0.086000;
            a1 =  0.110240;

            % Angle shorthand
            S2   = sin(q(2));  C2   = cos(q(2));
            S3   = sin(q(3));  C3   = cos(q(3));
            S4   = sin(q(4));  C4   = cos(q(4));
            S5   = sin(q(5));  C5   = cos(q(5));
            S23  = sin(q(2) - q(3));   % S_{2,−3}

            %% Category B3: L2, L3, L5 parallel/coplanar
            % Condition: S3^2 + S4^2 = 0  (Eq. 11)
            val_B3 = S3^2 + S4^2;
            if val_B3 < thr
                types{end+1} = 'B3 (L2,L3,L5 parallel: theta3~0, theta4~0)';
            end

            %% Category D3: L3,L4,L5 in plane; L1,L6 parallel to it
            % Condition: S(2-3)^2 + S4^2 + C5^2 = 0  (Eq. 17)
            val_D3 = S23^2 + S4^2 + C5^2;
            if val_D3 < thr
                types{end+1} = 'D3 (L3,L4,L5 coplanar; L1||L6)';
            end

            %% Category E: necessary condition is S4 = 0  (Eq. 19)
            if abs(S4) < sqrt(thr)
                types{end+1} = 'E (theta4 near 0 or pi — all-axes concurrent)';
            end

            %% Category B1: L1, L4, L6 parallel and coplanar  (Eq. 9)
            % Simplified check: the bracketed term + S(2-3)^2 + S5^2 = 0
            bracket_B1 = (a2*d5*S3 - a1*d5 - a5*d3)*C4 + a2*a5*S3 - a1*a5 + d3*d5;
            val_B1 = bracket_B1^2 + S23^2 + S5^2;
            if val_B1 < thr
                types{end+1} = 'B1 (L1,L4,L6 parallel/coplanar)';
            end

            %% Category C2: L1, L3, L4, L5 coplanar  (Eq. 13)
            % Condition: (d5*S4 - a5*C4)^2 + C5^2 + S3^2 = 0
            val_C2 = (d5*S4 - a5*C4)^2 + C5^2 + S3^2;
            if val_C2 < thr
                types{end+1} = 'C2 (L1,L3,L4,L5 coplanar)';
            end

            %% Category C3: L2, L3, L4, L6 coplanar  (Eq. 14)
            % Condition: (a1 - a2*S3)^2 + S(2-3)^2 + S4^2 = 0
            val_C3 = (a1 - a2*S3)^2 + S23^2 + S4^2;
            if val_C3 < thr
                types{end+1} = 'C3 (L2,L3,L4,L6 coplanar)';
            end

            if ~isempty(types)
                info.isSingular = true;
                info.type       = strjoin(types, ' | ');
            end
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %  QUINTIC POLYNOMIAL TRAJECTORY
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        function [torque_log, tau_force_log, time_log] = quinticTraj( ...
                sim, joints, joint_IDs, baseFrameID, endFrameID, ...
                penTipHandle, drawingHandle, targetOrient, F_magnitude, ...
                q_goal, T, n_steps, targetObjID, doPlot)
            % quinticTraj
            % Moves the robot from its CURRENT joint configuration to q_goal using
            % a smooth quintic polynomial profile on every joint. Contact force
            % torques are applied at each step 
            %
            % Inputs:
            %   sim           : CoppeliaSim remote API object
            %   joints        : URDF joint struct array
            %   joint_IDs     : cell array of joint handles {j1,...,j6}
            %   baseFrameID   : robot base frame handle
            %   endFrameID    : end-effector frame handle
            %   penTipHandle  : pen tip handle (for drawing lines)
            %   drawingHandle : CoppeliaSim drawing object handle
            %   targetOrient  : 1x3 [rx ry rz] desired EEF orientation ZYX [rad]
            %   F_magnitude   : contact force magnitude [N]
            %   q_goal        : 6x1 goal joint angles [rad]
            %                   pass [] to auto-solve via IK using targetObjID
            %   T             : motion duration [s]
            %   n_steps       : number of trajectory steps (resolution)
            %   targetObjID   : CoppeliaSim object handle for IK goal
            %                   pass -1 if supplying q_goal directly
            %   doPlot        : true/false — show real-time torque plot and
            %                   drawing object
            %
            % Outputs:
            %   torque_log    : n_steps x 6  (reserved, zeros)
            %   tau_force_log : n_steps x 6  contact torques  J^T * F
            %   time_log      : n_steps x 1  step indices

            nJoints = length(joint_IDs);   % 6 

            % Read current joint angles as start config
            q_start = zeros(nJoints, 1);
            for j = 1:nJoints
                q_start(j) = sim.getJointPosition(joint_IDs{j});
            end

       
            if isempty(q_goal)
                if targetObjID == -1
                    error('quinticTraj: supply either q_goal or TargetObjID.');
                end
                fprintf('quinticTraj: solving IK for goal pose...\n');
                q_goal = Final_functions.getJointAngles_IK( ...
                    sim, joints, targetObjID, baseFrameID, endFrameID, ...
                    joint_IDs, targetOrient);
            end
            %q_goal = q_goal(:);

            % Quintic coefficient matrix 
            % Boundary: q(0)=q0, q(T)=qf, q'(0)=q'(T)=0, q''(0)=q''(T)=0
            %
            %   q(t)   = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
            %   q'(t)  = a1 + 2*a2*t + 3*a3*t^2 + 4*a4*t^3 + 5*a5*t^4
            %   q''(t) = 2*a2 + 6*a3*t + 12*a4*t^2 + 20*a5*t^3

            M = [1,  0,    0,      0,       0,       0;
                1,  T,    T^2,    T^3,     T^4,     T^5;
                0,  1,    0,      0,       0,       0;
                0,  1,    2*T,    3*T^2,   4*T^3,   5*T^4;
                0,  0,    2,      0,       0,       0;
                0,  0,    2,      6*T,     12*T^2,  20*T^3];

            % Solve for each joint
            coeffs = zeros(nJoints, 6);
            for j = 1:nJoints
                b = [q_start(j); q_goal(j); 0; 0; 0; 0];
                coeffs(j,:) = (M \ b)';
            end

            
            t_vec = linspace(0, T, n_steps);

            % Pre-allocate logs 
            torque_log    = zeros(n_steps, nJoints);
            tau_force_log = zeros(n_steps, nJoints);
            time_log      = zeros(n_steps, 1);

            % Real-time plot
            if doPlot
                figure; clf; hold on; grid on;
                h_tau_f = gobjects(nJoints, 1);
                for k = 1:nJoints
                    h_tau_f(k) = plot(NaN, NaN, 'LineWidth', 1.5);
                end
                title('Quintic Traj — Contact Force Torques J^T*F (Nm)');
                leg_labels = arrayfun(@(k) sprintf('τ%d',k), 1:nJoints, 'UniformOutput', false);
                legend(leg_labels, 'Location', 'best');
                ylabel('Nm'); xlabel('Step');
                xlim([0, n_steps]);
            end

            prevPos = [];

            % Main trajectory loop
            for step = 1:n_steps
                tk = t_vec(step);

                % Evaluate quintic position
                q_cmd  = zeros(nJoints, 1);
                for j = 1:nJoints
                    a = coeffs(j,:);
                    q_cmd(j) = a(1) +   a(2)*tk +    a(3)*tk^2 + ...
                        a(4)*tk^3 + a(5)*tk^4 + a(6)*tk^5;
                end
                %q_cmd = wrapToPi(q_cmd);
                

                % Send commands
                for i = 1:nJoints
                    sim.setObjectInt32Param(joint_IDs{i}, sim.jointintparam_dynctrlmode, sim.jointdynctrl_position);
                    sim.setJointTargetPosition(joint_IDs{i}, q_cmd(i));
                end

                sim.step();

                % Read actual angles post-physics
                currJointAngles = zeros(nJoints, 1);
                for i = 1:nJoints
                    currJointAngles(i) = sim.getJointPosition(joint_IDs{i});
                end

                % Contact force torques  τ = J^T * F
                tau_force = Final_functions.getStaticJointTorques( ...
                    sim, currJointAngles, joints, F_magnitude, endFrameID);

                for i = 1:nJoints
                    sim.setJointTargetForce(joint_IDs{i}, tau_force(i));
                end

                % Log
                tau_force_log(step, :) = tau_force(:)';
                time_log(step)         = step;

                % Drawing line
                if doPlot
                    currPos = sim.getObjectPosition(penTipHandle, -1);
                    if ~isempty(prevPos)
                        sim.addDrawingObjectItem(drawingHandle, [prevPos, currPos]);
                    end
                    prevPos = currPos;
                    % plot updates also inside here
                end

                % Update real-time plot
                if doPlot
                    for k = 1:nJoints
                        set(h_tau_f(k), 'XData', time_log(1:step), ...
                            'YData', tau_force_log(1:step, k));
                    end
                    drawnow limitrate;
                end
            end
            tol_arrive = 0.01;
            max_wait   = 500;
            for w = 1:max_wait
                sim.step();
                arrived = true;
                for i = 1:nJoints
                    if abs(sim.getJointPosition(joint_IDs{i}) - q_goal(i)) > tol_arrive
                        arrived = false;
                        break;
                    end
                end
                if arrived
                    break;
                end
            end

            fprintf('quinticTraj: done. Steps=%d, Duration=%.2fs\n', n_steps, T);
        end
        

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % triangle function
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        function [torque_log, tau_force_log, time_log] = drawTriangle(sim, cy, cz, radius, wall, n_steps, ObjID, joint_IDs, joints, baseFrameID, endFrameID, penTipHandle, drawingHandle, targetOrient, F_magnitude)
            % Equilateral triangle centered at (cy, cz)
            % Vertices are equally spaced 120 degrees apart
            vertices = zeros(3, 2);
            for v = 1:3
                angle_v = pi/2 + (v-1) * (2*pi/3);
                vertices(v, 1) = cy + radius * cos(angle_v);  % y
                vertices(v, 2) = cz + radius * sin(angle_v);  % z
            end

            % Distribute steps across 3 sides
            steps_per_side = floor(n_steps / 3);
            total_steps    = steps_per_side * 3;

            % Pre-allocate logs
            torque_log    = zeros(total_steps, 6);
            tau_force_log = zeros(total_steps, 6);
            time_log      = zeros(total_steps, 1);

            % Move to first vertex using quinticTraj
            sim.setObjectPosition(ObjID, -1, [wall; vertices(1,1); vertices(1,2)]);
            sim.setObjectOrientation(ObjID, -1, [0; 0; 0]);

            Final_functions.quinticTraj(sim, joints, joint_IDs, baseFrameID, endFrameID, ...
                penTipHandle, drawingHandle, targetOrient, F_magnitude, ...
                [], 3, 50, ObjID, false);

            
            
            jointAngles_first = Final_functions.getJointAngles_IK(sim, joints, ObjID, baseFrameID, ...
                endFrameID, joint_IDs, targetOrient);

            for i = 1:6
                sim.setObjectInt32Param(joint_IDs{i}, sim.jointintparam_dynctrlmode, sim.jointdynctrl_position);
                sim.setJointTargetPosition(joint_IDs{i}, jointAngles_first(i));
            end



            % Capture start position only after robot has arrived
            prevPos = sim.getObjectPosition(penTipHandle, -1);

            % Real-time plot
            figure; clf;
            hold on; grid on;
            for k = 1:6
                h_tau_f(k) = plot(NaN, NaN, 'LineWidth', 1.5);
            end
            title('Joint Torques from Contact Force J^T*F (Nm)');
            legend('τ1','τ2','τ3','τ4','τ5','τ6','Location','best');
            ylabel('Nm'); xlabel('Step');
            xlim([0, total_steps]);

            step = 0;

            for side = 1:3
                v_start = vertices(side, :);
                v_end   = vertices(mod(side, 3) + 1, :);

                for s = 1:steps_per_side
                    step = step + 1;
                    t    = (s - 1) / steps_per_side;

                    y = v_start(1) + t * (v_end(1) - v_start(1));
                    z = v_start(2) + t * (v_end(2) - v_start(2));

                    sim.setObjectPosition(ObjID, -1, [wall; y; z]);
                    sim.setObjectOrientation(ObjID, -1, [0; 0; 0]);

                    jointAngles = Final_functions.getJointAngles_IK(sim, joints, ObjID, baseFrameID, endFrameID, joint_IDs, targetOrient);

                    % Position control so joints actually track
                    for i = 1:6
                        sim.setObjectInt32Param(joint_IDs{i}, sim.jointintparam_dynctrlmode, sim.jointdynctrl_position);
                        sim.setJointTargetPosition(joint_IDs{i}, jointAngles(i));
                    end

                    

                    % Switch to force mode for torque logging
                    currJointAngles = zeros(1, 6);
                    for i = 1:6
                        currJointAngles(i) = sim.getJointPosition(joint_IDs{i});
                        sim.setObjectInt32Param(joint_IDs{i}, sim.jointintparam_dynctrlmode, sim.jointdynctrl_force);
                        sim.setJointTargetForce(joint_IDs{i}, 0);
                    end

                    % Compute and apply contact torques
                    tau_force = Final_functions.getStaticJointTorques(sim, currJointAngles, joints, F_magnitude, endFrameID);
                    for i = 1:6
                        sim.setJointTargetForce(joint_IDs{i}, tau_force(i));
                    end

                    % Log
                    tau_force_log(step, :) = tau_force(:)';
                    time_log(step)         = step;

                    % Drawing
                    currPos = sim.getObjectPosition(penTipHandle, -1);
                    if ~isempty(prevPos)
                        sim.addDrawingObjectItem(drawingHandle, [prevPos, currPos]);
                    end
                    prevPos = currPos;

                    % Update plot
                    for k = 1:6
                        set(h_tau_f(k), 'XData', time_log(1:step), 'YData', tau_force_log(1:step, k));
                    end
                    drawnow limitrate;
                end
            end

            % Close triangle back to first vertex
            sim.setObjectPosition(ObjID, -1, [wall; vertices(1,1); vertices(1,2)]);
            sim.setObjectOrientation(ObjID, -1, [0; 0; 0]);
            jointAngles = Final_functions.getJointAngles_IK(sim, joints, ObjID, baseFrameID, endFrameID, joint_IDs, targetOrient);
            for i = 1:6
                sim.setObjectInt32Param(joint_IDs{i}, sim.jointintparam_dynctrlmode, sim.jointdynctrl_position);
                sim.setJointTargetPosition(joint_IDs{i}, jointAngles(i));
            end
            for w = 1:200
                sim.step();
                arrived = true;
                for i = 1:6
                    if abs(sim.getJointPosition(joint_IDs{i}) - jointAngles(i)) > tol_arrive
                        arrived = false;
                        break;
                    end
                end
                if arrived; break; end
                closingPos = sim.getObjectPosition(penTipHandle, -1);
                sim.addDrawingObjectItem(drawingHandle, [prevPos, closingPos]);
                prevPos = closingPos;
            end
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Draw circle function (Wall)
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        function [torque_log, tau_force_log, time_log] = drawCircle(sim, cy, cz, radius, wall, n_steps, ObjID, joint_IDs, joints, baseFrameID, endFrameID, penTipHandle, drawingHandle, targetOrient, F_magnitude)

            angles = linspace(0, 2*pi, n_steps);
            prevPos = [];

            % Pre-allocate logs
            torque_log     = zeros(n_steps, 6);
            tau_force_log  = zeros(n_steps, 6);
            time_log       = zeros(n_steps, 1);

            % Smoothly move to the first circle position using quinticTraj
            first_y = cy + radius * cos(angles(1));
            first_z = cz + radius * sin(angles(1));
            sim.setObjectPosition(ObjID, -1, [wall; first_y; first_z]);
            sim.setObjectOrientation(ObjID, -1, [0; 0; 0]);

            Final_functions.quinticTraj(sim, joints, joint_IDs, baseFrameID, endFrameID, ...
                penTipHandle, drawingHandle, targetOrient, F_magnitude, ...
                [], 3, 50, ObjID, false);   % false = don't draw during approach

            % Explicitly block until robot is at first circle position
            
            jointAngles_first = Final_functions.getJointAngles_IK(sim, joints, ObjID, baseFrameID, ...
                endFrameID, joint_IDs, targetOrient);

            for i = 1:6
                sim.setObjectInt32Param(joint_IDs{i}, sim.jointintparam_dynctrlmode, sim.jointdynctrl_position);
                sim.setJointTargetPosition(joint_IDs{i}, jointAngles_first(i));
            end

            
            
            % Capture starting pen position BEFORE loop begins
            prevPos = sim.getObjectPosition(penTipHandle, -1);

            % Real-time Plot Initialization 
            figure; clf;
 
            hold on; grid on;
            for k = 1:6
                h_tau_f(k) = plot(NaN, NaN, 'LineWidth', 1.5);
            end
            title('Joint Torques from Contact Force J^T*F (Nm)'); 
            legend('τ1','τ2','τ3','τ4','τ5','τ6','Location','best');
            ylabel('Nm'); xlabel('Step');
            xlim([0, n_steps]);

            pause(2);

            for step = 1:n_steps
                y = cy + radius * cos(angles(step));
                z = cz + radius * sin(angles(step));

                sim.setObjectPosition(ObjID, -1, [wall; y; z]);
                sim.setObjectOrientation(ObjID, -1, [0; 0; 0]);

                jointAngles = Final_functions.getJointAngles_IK(sim, joints, ObjID, baseFrameID, endFrameID, joint_IDs, targetOrient);

                % Switch to position control so joints actually track
                for i = 1:6
                    sim.setObjectInt32Param(joint_IDs{i}, sim.jointintparam_dynctrlmode, sim.jointdynctrl_position);
                    sim.setJointTargetPosition(joint_IDs{i}, jointAngles(i));
                end

                % Step until joints have actually reached the target
                for c = 1:20
                    sim.step();
                end

                % Switch back to force mode for torque logging
                currJointAngles = zeros(1,6);
                for i = 1:6
                    currJointAngles(i) = sim.getJointPosition(joint_IDs{i});
                    sim.setObjectInt32Param(joint_IDs{i}, sim.jointintparam_dynctrlmode, sim.jointdynctrl_force);
                    sim.setJointTargetForce(joint_IDs{i}, 0);
                end

                % Compute and apply gravity + contact torques
                tau_force = Final_functions.getStaticJointTorques(sim, currJointAngles, joints, F_magnitude, endFrameID);
                for i = 1:6
                    sim.setJointTargetForce(joint_IDs{i}, tau_force(i));
                end

                % Log
                tau_force_log(step, :) = tau_force(:)';
                time_log(step)         = step;

                % Drawing - pen tip now matches actual robot position
                currPos = sim.getObjectPosition(penTipHandle, -1);
                if ~isempty(prevPos)
                    sim.addDrawingObjectItem(drawingHandle, [prevPos, currPos]);
                end
                prevPos = currPos;

                % Update plots
                for k = 1:6
                    set(h_tau_f(k), 'XData', time_log(1:step), 'YData', tau_force_log(1:step, k));
                end
                drawnow limitrate;
            end

            % Close circle - return to first point with multiple steps
            sim.setObjectPosition(ObjID, -1, [wall; first_y; first_z]);
            sim.setObjectOrientation(ObjID, -1, [0; 0; 0]);
            jointAngles = Final_functions.getJointAngles_IK(sim, joints, ObjID, baseFrameID, endFrameID, joint_IDs, targetOrient);
            for i = 1:6
                sim.setObjectInt32Param(joint_IDs{i}, sim.jointintparam_dynctrlmode, sim.jointdynctrl_position);
                sim.setJointTargetPosition(joint_IDs{i}, jointAngles(i));
            end
            for c = 1:20
                sim.step();
                closingPos = sim.getObjectPosition(penTipHandle, -1);
                sim.addDrawingObjectItem(drawingHandle, [prevPos, closingPos]);
                prevPos = closingPos;
            end
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%
        % circle floor
        %%%%%%%%%%%%%%%%%%%%%%%%%%

        function [torque_log, tau_force_log, time_log] = drawCircleFloor(sim, cx, cy, radius, floor_z, n_steps, ObjID, joint_IDs, joints, baseFrameID, endFrameID, penTipHandle, drawingHandle, targetOrient, F_magnitude)
            % drawCircleFloor
            % Draws a circle on a flat horizontal surface (floor) centered at (cx, cy)
            % at height floor_z. End effector points downward.
           

            angles = linspace(0, 2*pi, n_steps);
            prevPos = [];

            % Pre-allocate logs
            torque_log     = zeros(n_steps, 6);
            tau_force_log  = zeros(n_steps, 6);
            time_log       = zeros(n_steps, 1);

            % Smoothly move to the first circle position using quinticTraj
            first_x = cx + radius * cos(angles(1));
            first_y = cy + radius * sin(angles(1));
            sim.setObjectPosition(ObjID, -1, [first_x; first_y; floor_z]);
            sim.setObjectOrientation(ObjID, -1, [0; 0; 0]);

            Final_functions.quinticTraj(sim, joints, joint_IDs, baseFrameID, endFrameID, ...
                penTipHandle, drawingHandle, targetOrient, F_magnitude, ...
                [], 3, 50, ObjID, false);   % false = don't draw during approach

            % Capture starting pen position BEFORE loop begins
            prevPos = sim.getObjectPosition(penTipHandle, -1);

            % Real-time plot
            figure; clf;
            hold on; grid on;
            for k = 1:6
                h_tau_f(k) = plot(NaN, NaN, 'LineWidth', 1.5);
            end
            title('Joint Torques from Contact Force J^T*F (Nm)');
            legend('τ1','τ2','τ3','τ4','τ5','τ6','Location','best');
            ylabel('Nm'); xlabel('Step');
            xlim([0, n_steps]);

            prevPos = [];

            pause(3);

            for step = 1:n_steps
                % Circle in the horizontal XY plane — Z is fixed at floor height
                x = cx + radius * cos(angles(step));
                y = cy + radius * sin(angles(step));

                sim.setObjectPosition(ObjID, -1, [x; y; floor_z]);
                sim.setObjectOrientation(ObjID, -1, [0; 0; 0]);

                jointAngles = Final_functions.getJointAngles_IK(sim, joints, ObjID, baseFrameID, endFrameID, joint_IDs, targetOrient);

                % Apply joint targets
                for i = 1:6
                    sim.setObjectInt32Param(joint_IDs{i}, sim.jointintparam_dynctrlmode, sim.jointdynctrl_force);
                    sim.setJointPosition(joint_IDs{i},       jointAngles(i));
                    sim.setJointTargetPosition(joint_IDs{i}, jointAngles(i));
                    sim.setJointTargetVelocity(joint_IDs{i}, 0);
                    sim.setJointTargetForce(joint_IDs{i},    0);
                end

                sim.step();
                

                % Read joint angles after step
                currJointAngles = zeros(1, 6);
                for i = 1:6
                    currJointAngles(i) = sim.getJointPosition(joint_IDs{i});
                end

                % Compute contact torques — force is now in -Z direction (into floor)
                tau_force = Final_functions.getStaticJointTorques(sim, currJointAngles, joints, F_magnitude, endFrameID);
                for i = 1:6
                    sim.setJointTargetForce(joint_IDs{i}, tau_force(i));
                end

                % Log
                tau_force_log(step, :) = tau_force(:)';
                time_log(step)         = step;

                % Drawing
                currPos = sim.getObjectPosition(penTipHandle, -1);
                if ~isempty(prevPos)
                    sim.addDrawingObjectItem(drawingHandle, [prevPos, currPos]);
                end
                prevPos = currPos;

                % Update plot
                for k = 1:6
                    set(h_tau_f(k), 'XData', time_log(1:step), 'YData', tau_force_log(1:step, k));
                end
                drawnow limitrate;
            end

            % Close circle - step past the start point to ensure no gap
            close_angles = linspace(2*pi, 2*pi + 0.8, 10);  % 0.3 rad past start
            for c = 1:length(close_angles)
                x_c = cx + radius * cos(close_angles(c));
                y_c = cy + radius * sin(close_angles(c));

                sim.setObjectPosition(ObjID, -1, [x_c; y_c; floor_z]);
                sim.setObjectOrientation(ObjID, -1, [0; 0; 0]);

                jointAngles = Final_functions.getJointAngles_IK(sim, joints, ObjID, baseFrameID, endFrameID, joint_IDs, targetOrient);
                for i = 1:6
                    sim.setObjectInt32Param(joint_IDs{i}, sim.jointintparam_dynctrlmode, sim.jointdynctrl_position);
                    sim.setJointTargetPosition(joint_IDs{i}, jointAngles(i));
                end

                sim.step();

                closingPos = sim.getObjectPosition(penTipHandle, -1);
                sim.addDrawingObjectItem(drawingHandle, [prevPos, closingPos]);
                prevPos = closingPos;
            end
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%
        % TORQUES
        %%%%%%%%%%%%%%%%%%%%%%%%%%

        function torques = getStaticJointTorques(sim, jointAngles, joints, Fz_in_EndFrame, endFrameID)
            % getStaticJointTorques
            % Computes the joint torques required to statically balance an external
            % force applied along the z-axis of the end-effector frame.
            %
            % Inputs:
            %   sim              : CoppeliaSim remote API object
            %   jointAngles      : 6×1 or 1×6 vector of joint angles [rad]
            %   joints           : Struct array of joint parameters (parsed from URDF)
            %   Fz_in_EndFrame   : Scalar force magnitude applied along the end-frame z-axis [N]
            %   endFrameID       : Handle of the end-effector frame in CoppeliaSim
            %
            % Output:
            %   torques          : 6×1 vector of joint torques required for static equilibrium
            torques = zeros(6,1);

            T_world_endFrame = getObjectTmat(sim, -1, endFrameID);
            F = Fz_in_EndFrame * T_world_endFrame(1:3, 3); % world force direction
            J_current = getJacobian(jointAngles, joints);
            % Build wrench and compute joint torques
            wrench = [-F; 0; 0; 0]; % (Fx,Fy,Fz,Mx,My,Mz) in world

            torques = J_current' * wrench; % 6x1

        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % elbow up
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        function setElbowUp(sim, ur5e_joint_IDs, jaco_joint_IDs)
            

            %% ── UR5e elbow-up configuration ─────────────────────────────────────────
            % Joint order: base, shoulder, elbow, wrist1, wrist2, wrist3
            % Elbow-up means shoulder pulls back (negative) and elbow bends upward
            ur5e_home = [pi/4; pi; -pi/1.5; pi/4; pi/2; 0];

            %% ── Jaco elbow-up configuration ─────────────────────────────────────────
            % Chosen to satisfy all of:
            %   theta4 = pi/2  → sin(theta4)=1,  avoids E  (sin~0)
            %   theta5 = pi/4  → sin(theta5)>0,  avoids B1 (sin~0)
            %   theta3 != 0    →                 avoids C2  (sin~0)
            %   theta2 != theta3 →               avoids B1  (sin(2-3)~0)
            % Arm pointing down for floor drawing
            jaco_home = [0; 1.2*pi; pi/4; 0; 0; 0];

            % Apply to UR5e
            fprintf('Moving UR5e to elbow-up home...\n');
            for j = 1:6
                sim.setJointPosition(ur5e_joint_IDs{j},       ur5e_home(j));
                sim.setJointTargetPosition(ur5e_joint_IDs{j}, ur5e_home(j));
                sim.setJointTargetVelocity(ur5e_joint_IDs{j}, 0);
            end

            % Apply to Jaco
            fprintf('Moving Jaco to elbow-up home...\n');
            for j = 1:6
                sim.setJointPosition(jaco_joint_IDs{j},       jaco_home(j));
                sim.setJointTargetPosition(jaco_joint_IDs{j}, jaco_home(j));
                sim.setJointTargetVelocity(jaco_joint_IDs{j}, 0);
            end

            % Step sim to apply positions
            sim.step();
            fprintf('Both arms at elbow-up home position.\n');
        end
    end
end




%% HELPER FUNCTIONS

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

