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
        s=0;
        s_vel = 0;
        input_pathlog
        snake_pathlog
        e_rpylog
        %表示をする次元
        dim
    end

    methods
        function obj = SnakeRobot(num_joint_, length_quater_, alpha_yaw_, alpha_pitch_, dim_)
            obj.num_joint = num_joint_;
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
        end
        
        function updateModel(obj)
            snake_model = snakeModel(obj);
            if obj.dim == 2
                obj.snake_pathlog = [obj.snake_pathlog; snake_model(:,1:2)];
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
        %function snake_rad = snake2Rad(obj, s_h_)
        %    snake_rad = ;
        %end
        
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