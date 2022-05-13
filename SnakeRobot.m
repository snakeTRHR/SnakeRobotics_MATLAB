classdef SnakeRobot < handle
    properties
        Model_C
        Model_init
        num_joint
        %体形サーペノイド波四分の一長さ
        length_quater
        %サーペノイド波一つ分x軸長さ
        length_cycle
        %各ジョイント長さ
        length_joint
        alpha_yaw
        alpha_pitch
        bias_yaw = 0;
        bias_pitch = 0;
        curvature_yaw = 0;
        curvature_pitch = 0;
        torsion = 0;
        s = 0;
        s_vel = 0;
        input_pathlog
        snake_pathlog
        e_rpylog
        %表示をする次元
        dim
        %離散化データ
        %各関節角度[x1 y1; x2 y2; ...]
        joint_rad_y_now = 0;
        joint_rad_p_now = 0;
        joint_rad_y_ini = NaN;
        joint_rad_p_ini = NaN;
        joint_radlog_y
        joint_radlog_p
        joint_radlog
        pos_discretization
        s_y_last = 0;
        s_p_last = 0;
        discretization_pathlog
        
    end

    methods
        function obj = SnakeRobot(num_joint_, length_joint, length_quater_, alpha_yaw_, alpha_pitch_, dim_)
            obj.num_joint = num_joint_;
            obj.length_joint = length_joint;
            obj.dim = dim_;
            obj.changeSnakeParam(alpha_yaw_, alpha_pitch_, length_quater_);
            theta_roll = 0;
            theta_pitch = -1*alpha_pitch_;
            theta_yaw = alpha_yaw_;
            obj.Model_C = zeros(3, 1);
            Identity_Matrix = eye(3).*-1;
            Identity_Matrix(1,1)=1;
            Rotation_Matrix = [cos(theta_pitch)*cos(theta_yaw)+sin(theta_pitch)*sin(theta_roll)*sin(theta_yaw),-cos(theta_pitch)*sin(theta_yaw)+sin(theta_pitch)*sin(theta_roll)*cos(theta_yaw),sin(theta_pitch)*cos(theta_roll);
                               cos(theta_roll)*sin(theta_yaw),                                                  cos(theta_roll)*cos(theta_yaw),                                                -sin(theta_roll);
                              -sin(theta_pitch)*cos(theta_yaw)+cos(theta_pitch)*sin(theta_roll)*sin(theta_yaw), sin(theta_pitch)*sin(theta_yaw)+cos(theta_pitch)*sin(theta_roll)*cos(theta_yaw),cos(theta_pitch)*cos(theta_roll)];
            Initial_Matrix = (Rotation_Matrix*Identity_Matrix)';
            obj.Model_init=[obj.Model_C; reshape(Initial_Matrix, 9, 1)];
            if obj.dim == 2
                obj.snake_pathlog=(obj.Model_C(1:2,1))';
            elseif obj.dim == 3
                obj.snake_pathlog = obj.Model_C';
            end
            %離散モデルと連続体モデルのs=0を原点に一致させる
            obj.discretization_pathlog=[0, 0, 0];
            %離散モデルと連続体モデルの初期姿勢を一致させる
            obj.joint_rad_y_ini=theta_yaw;
            obj.joint_rad_p_ini=theta_pitch;
            obj.joint_rad_y_now=theta_yaw;
            obj.joint_rad_p_now=theta_pitch;
            %obj.joint_radlog=0;
            obj.joint_radlog_y=obj.joint_rad_y_ini;
            obj.joint_radlog_p=obj.joint_rad_p_ini;
            obj.s_y_last=obj.length_joint;
            %obj.joint_radlog=0;
        end
%         
%         function updateModel(obj)
%             snake_model = snakeModel(obj);
%             if obj.dim == 2
%                 %連続体モデルを解く
%                 obj.snake_pathlog = [obj.snake_pathlog; snake_model(:,1:2)];
%                 %離散化
%                 %+微小Θ
%                 obj.joint_rad_y_now = obj.joint_rad_y_now+obj.snake2RadY();
%                 %初期値の設定
%                 if size(obj.joint_radlog_y, 1)==1 && obj.joint_radlog_y(1, 1)==obj.joint_rad_y_ini
%                     if(obj.s - obj.s_last) >= obj.length_joint/2
%                         obj.joint_radlog_y(1, 1)=obj.joint_rad_y_now;
%                         obj.s_last = obj.s;
%                     end
%                 else
%                     %一関節分進んだら角度を更新する
%                     if (obj.s - obj.s_last) >= obj.length_joint
%                         obj.joint_radlog_y = [obj.joint_radlog_y; obj.joint_rad_y_now];
%                         obj.s_last = obj.s;
%                     end
%                 end
%             elseif obj.dim == 3
%                 obj.snake_pathlog = [obj.snake_pathlog; snake_model];
%             end
%         end
%         function updateModel(obj)
%             snake_model = snakeModel(obj);
%             if obj.dim == 2
%                 %連続体モデルを解く
%                 obj.snake_pathlog = [obj.snake_pathlog; snake_model(:,1:2)];
%                 %離散化
%                 %+微小Θ
%                 obj.joint_rad_y_now = obj.snake2RadY();
%                 %初期値の設定
%                 if size(obj.joint_radlog_y, 1)==1 && obj.joint_radlog_y(1, 1)==obj.joint_rad_y_ini
%                     if(obj.s - obj.s_last) >= obj.length_joint/2
%                         obj.joint_radlog_y(1, 1)=obj.joint_rad_y_ini+obj.joint_rad_y_now*obj.length_joint;
%                         obj.s_last = obj.s+obj.length_joint/2;
%                     end
%                 else
%                     %一関節分進んだら角度を更新する
%                     if (obj.s - obj.s_last) >= obj.length_joint/2
%                         obj.joint_radlog_y = [obj.joint_radlog_y; obj.joint_radlog_y(end, 1)+obj.joint_rad_y_now*obj.length_joint];
%                         obj.s_last = obj.s+obj.length_joint/2;
%                     end
%                 end
%             elseif obj.dim == 3
%                 obj.snake_pathlog = [obj.snake_pathlog; snake_model];
%             end
%         end
%         function updateModel(obj)
%             snake_model = snakeModel(obj);
%             if obj.dim == 2
%                 %連続体モデルを解く
%                 obj.snake_pathlog = [obj.snake_pathlog; snake_model(:,1:2)];
%                 %離散化
%                 %+微小Θ
%                 obj.joint_rad_y_now = obj.snake2RadY();
%                 %初期値の設定
%                 if size(obj.joint_radlog_y, 1)==1 && obj.joint_radlog_y(1, 1)==obj.joint_rad_y_ini
%                     if(obj.s - obj.s_last) >= 2*obj.length_joint
%                         obj.joint_radlog_y(1, 1)=obj.joint_rad_y_ini-obj.snakeCurvatureYaw(obj.s)*2*obj.length_joint;
%                         disp([obj.joint_rad_y_now*2*obj.length_joint*180/pi, -obj.snakeCurvatureYaw(obj.s), -obj.snakeCurvatureYaw(obj.s)*2*obj.length_joint*180/pi, obj.s])
%                         obj.s_last = obj.s;
%                     end
%                 else
%                     %一関節分進んだら角度を更新する
%                     if (obj.s - obj.s_last) >= 2*obj.length_joint
%                         obj.joint_radlog_y = [obj.joint_radlog_y; obj.joint_radlog_y(end, 1)+obj.joint_rad_y_now*2*obj.length_joint];
%                         disp(obj.joint_rad_y_now*2*obj.length_joint*180/pi)
%                         obj.s_last = obj.s;
%                     end
%                 end
%             elseif obj.dim == 3
%                 obj.snake_pathlog = [obj.snake_pathlog; snake_model];
%             end
%         end
        function updateModel(obj)
            snake_model = snakeModel(obj);
            if obj.dim == 2
                %連続体モデルを解く
                obj.snake_pathlog = [obj.snake_pathlog; snake_model(:,1:2)];

                %離散化
                %+微小Θ
                obj.joint_rad_p_now = obj.joint_rad_p_now + obj.snake2RadP();
                temp_rady = obj.snake2RadY();
                obj.joint_rad_y_now = obj.joint_rad_y_now + temp_rady;
                disp([obj.s, obj.joint_rad_p_now, obj.joint_rad_y_now, temp_rady])
                %現在計算中の関節
                joint_num_now = size(obj.joint_radlog, 1)+1;
                %0番目の関節(絶対角のときは必要(相対角の場合はいらない))
                if (obj.s >= obj.length_joint) && (size(obj.joint_radlog, 1) == 0)
                    obj.joint_radlog=obj.joint_rad_y_now;
                    disp(obj.joint_radlog)
                    obj.s_y_last=obj.s;
                end
                %奇数番目の関節
                if ((rem(joint_num_now, 2) == 0) && ((obj.s-obj.s_p_last) >= 2*obj.length_joint))
                    obj.joint_radlog = [obj.joint_radlog;
                                        obj.joint_rad_p_now];
                    disp([obj.s, obj.s_p_last, obj.joint_rad_p_now])
                    %obj.joint_rad_p_now = 0;
                    obj.s_p_last = obj.s;
                end
                disp(obj.s)
                %偶数番目の関節
                if ((rem(joint_num_now, 2) == 1) && ((obj.s-obj.s_y_last) >= 2*obj.length_joint))
                    obj.joint_radlog = [obj.joint_radlog;
                                        obj.joint_rad_y_now];
                    disp([obj.s, obj.s_y_last, obj.joint_rad_y_now])
                    disp(obj.joint_radlog)
                    %obj.joint_rad_y_now = 0;
                    obj.s_y_last = obj.s;
                end
            elseif obj.dim == 3
                obj.snake_pathlog = [obj.snake_pathlog; snake_model];
            end
        end
        %連続体モデルの連立微分方程式
        function model = snakeModel(obj)
            temp_s = obj.s + obj.s_vel;
            [~,serpen] = ode45(@obj.serpenOde, [obj.s temp_s], obj.Model_init);
            obj.s = temp_s;
            obj.Model_init = (serpen(end, :))';
            model = serpen(:,1:3);
        end
        
        function dydt = serpenOde(obj, t, y)
            dydt = [y(4);
                    y(5);
                    y(6);
                    obj.snakeCurvatureYaw(t)*y(7)-obj.snakeCurvaturePitch(t)*y(10);
                    obj.snakeCurvatureYaw(t)*y(8)-obj.snakeCurvaturePitch(t)*y(11);
                    obj.snakeCurvatureYaw(t)*y(9)-obj.snakeCurvaturePitch(t)*y(12);
                   -obj.snakeCurvatureYaw(t)*y(4)+obj.snakeTorsion(t)*y(10);
                   -obj.snakeCurvatureYaw(t)*y(5)+obj.snakeTorsion(t)*y(11);
                   -obj.snakeCurvatureYaw(t)*y(6)+obj.snakeTorsion(t)*y(12);
                    obj.snakeCurvaturePitch(t)*y(4)-obj.snakeTorsion(t)*y(7);
                    obj.snakeCurvaturePitch(t)*y(5)-obj.snakeTorsion(t)*y(8);
                    obj.snakeCurvaturePitch(t)*y(6)-obj.snakeTorsion(t)*y(9)];
        end
 
        function curvature_yaw = snakeCurvatureYaw(obj, s_)
            curvature_yaw = obj.alpha_yaw*pi*sin(s_*pi/(2*obj.length_quater))/(2*obj.length_quater)+obj.bias_yaw;
            disp([curvature_yaw, obj.alpha_yaw, obj.length_quater, (2*obj.length_quater), sin(s_*pi/(2*obj.length_quater))])
        end

        function curvature_pitch = snakeCurvaturePitch(obj, s_)
            curvature_pitch = 0;
        end
         
        function torsion = snakeTorsion(obj, s_)
            torsion = 0;
        end
        
        function changeSnakeParam(obj,  alpha_yaw_, alpha_pitch_, length_quarter_)
            obj.alpha_yaw = alpha_yaw_;
            obj.alpha_pitch = alpha_pitch_;
            obj.length_quater = length_quarter_;
            obj.length_cycle = obj.calCycleLength(alpha_yaw_, alpha_pitch_, length_quarter_);
        end

        function changeAlphaYaw(obj, alpha_yaw_)
            obj.alpha_yaw = alpha_yaw_;
        end

        %蛇の一周期で移動する距離(bias=0のx軸長さ)
        function L = calCycleLength(obj, alpha_yaw_, alpha_pitch_, length_quater_)
            syms s_;
            sn = cos(alpha_yaw_*cos(s_*pi/(2*length_quater_)));
            L = double(int(sn, 0, 4*length_quater_));
        end

        
        %角度に変換
        function joint_rad_p = snake2RadP(obj)
            ds=obj.s_vel;
            joint_rad_p=0;
            %角度の回転方向が逆方向なのでマイナスをつける
            for i=obj.s-ds:0.001:obj.s
                joint_rad_p=joint_rad_p-obj.snakeCurvaturePitch(i)*0.001;
            end
        end
        function joint_rad_y = snake2RadY(obj)
            ds=obj.s_vel;
            %角度の回転方向が逆方向なのでマイナスをつける
            joint_rad_y=0;
            for i=obj.s-ds:0.001:obj.s
                joint_rad_y=joint_rad_y-obj.snakeCurvatureYaw(i)*0.001;
            end

            disp([ -obj.snakeCurvatureYaw(obj.s), ds])
        end

        
%         function calDiscretization(obj)
%             %離散化した座標logを求める
%             if obj.dim == 2
%                 for i = 1:size(obj.joint_radlog_y, 1)
%                     temp_discretization_x = obj.discretization_pathlog(end, 1)+obj.length_joint*cos(obj.joint_radlog_y(i, 1));
%                     temp_discretization_y = obj.discretization_pathlog(end, 2)+obj.length_joint*sin(obj.joint_radlog_y(i, 1));
%                     obj.discretization_pathlog = [obj.discretization_pathlog;
%                                                   temp_discretization_x, temp_discretization_y, 0];
%                 end
%             elseif obj.dim == 3
%             end
%         end
%         function calDiscretization(obj)
%             %離散化した座標logを求める
%             if obj.dim == 2
%                 for i = 1:size(obj.joint_radlog_y, 1)
%                     temp_discretization_x = obj.discretization_pathlog(end, 1)+2*obj.length_joint*cos(obj.joint_radlog_y(i, 1));
%                     temp_discretization_y = obj.discretization_pathlog(end, 2)+2*obj.length_joint*sin(obj.joint_radlog_y(i, 1));
%                     obj.discretization_pathlog = [obj.discretization_pathlog;
%                                                   temp_discretization_x, temp_discretization_y, 0];
%                 end
%             elseif obj.dim == 3
%             end
%         end
        function calDiscretization(obj)
            %離散化した座標logを求める
            if obj.dim == 2
                for i = 1:size(obj.joint_radlog, 1)
                    if rem(i, 2) == 1
                        temp_discretization_x = obj.discretization_pathlog(end, 1)+2*obj.length_joint*cos(obj.joint_radlog(i, 1));
                        temp_discretization_y = obj.discretization_pathlog(end, 2)+2*obj.length_joint*sin(obj.joint_radlog(i, 1));
                        obj.discretization_pathlog = [obj.discretization_pathlog;
                                                      temp_discretization_x, temp_discretization_y, 0];
                    end
                end
            elseif obj.dim == 3
            end
        end
        %生成した経路と各パラメータの変換
        function path2Param(obj, v_, bias_yaw_center_)
            obj.s_vel = 4*obj.length_quater*v_/obj.length_cycle;
            obj.bias_yaw = obj.length_cycle*bias_yaw_center_/(4*obj.length_quater);
            %{
            disp("bias_yaw")
            disp(obj.bias_yaw)
            disp("bias_yaw_end")
            %}
        end
        
        function changeVel(obj, s_vel_)
            obj.s_vel = s_vel_;
        end

        %log作成
        function createLog(obj, path_)
            obj.input_pathlog = [obj.input_pathlog; path_];
        end

        %表示
        function plotSnake(obj)
            if obj.dim == 2
                %plot(obj.input_pathlog(:,1), obj.input_pathlog(:,2))
                %hold on
                disp(obj.snake_pathlog)
                plot(obj.snake_pathlog(:,1), obj.snake_pathlog(:,2))
                grid on
                axis equal
                %hold off
            elseif obj.dim == 3
                plot3(obj.input_pathlog(:,1), obj.input_pathlog(:,2), obj.input_pathlog(:,3))
                hold on
                plot3(obj.snake_pathlog(:,1), obj.snake_pathlog(:,2), obj.snake_pathlog(:,3))
                hold off
            end
        end

        function data = plotData(obj)
            if obj.dim == 2
                data = obj.snake_pathlog(:, 1:2);
            elseif obj.dim == 3
                data = obj.snake_pathlog(:, 1:3);
            end
        end
    end
end