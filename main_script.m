clc
clear

% prompting input from user
prompt = {'Attacker position x,y','Defender position in degree','Maximum attacker speed','Maximum defender speed'};
dlgtitle = 'Enter input parameters';
input_data = inputdlg(prompt,dlgtitle);
atk_pos = str2num(input_data{1})'; % initial attack position
init_def_angle = str2num(input_data{2}); % initial defender angle from +ve x-axis
max_atk_speed = str2num(input_data{3}); % maximum attacker speed
max_def_speed = str2num(input_data{4}); % maximum defender speed

r = 1; % radius of protected circle
cir_cent = [0 0]'; % center coordinates of the protected circle
dt = 0.01; % time difference
integral = 0;
k_p = 1; % proportinal gain
k_i = 100; % integral gain
k_d = 0.001; % differential gain
prev_error = 0; % tracking previous error for controller

def_pos = [r*cosd(init_def_angle);r*sind(init_def_angle)]; % initial defender position
v = max_atk_speed/max_def_speed;
init_atk_pos = atk_pos;

%calculates distance of attacker from center of the circle
init_atk_dist = dist_calc(atk_pos,cir_cent);

% caculates optimal heading direction of attacker in degree
phi = asind(v/init_atk_dist);

% for loop for animating the defender and attacker motion
for t = 1:2000
    clf
    viscircles(cir_cent',1,'LineWidth',2,'color','green'); % plots protected-circle
    viscircles(cir_cent',v,'LineWidth',1,'LineStyle','--','color','black'); % plots v-circle
    axis(gca, 'equal')
    axis([-5 5 -5 5])
    hold on 
    
    theta = angle(atk_pos,def_pos); 
    def_angle = atan2d(def_pos(2),def_pos(1));
    def_ang_rad = def_angle*(3.14/180);
    Rot_mat = [cosd(def_angle) -sind(def_angle); sind(def_angle) cosd(def_angle)];
    
    % plot of winning region
    syms x y
    xx = x*cosd(def_angle)+y*sind(def_angle);
    yy = -x*sind(def_angle)+y*cosd(def_angle);
    V = -sqrt(((sqrt(xx^2+yy^2)/v)^2)-1)+acos(v/sqrt(xx^2+yy^2))+abs(atan2(yy,xx))-acos(v)+sqrt((1/v^2)-1);
    fimplicit(V,'LineWidth',2,'color','red')
    
    % plotting the afferent and dispersal surfaces
    a = [-1 -8 1 8];
    b = [0 0 0 0];
    for l = 1:4
        vector = [a(l);b(l)];
        Rot_vect = Rot_mat * vector;
        if(l>2)
            e(l-2) = Rot_vect(1);
            f(l-2) = Rot_vect(2);
        end
        if(l<=2)
            c(l) = Rot_vect(1);
            d(l) = Rot_vect(2);
        end            
    end      
    plot(e,f,'LineWidth',2,'color','black')
    plot(c,d,'LineWidth',2,'color','black')
    
    % function call to obtain tangent point at v-circle
    [tan_point1,tan_point2] = tangent_point(atk_pos,cir_cent,v);
    
    if (t == 1)
        % dummy_x and dummy_y assit in obtaining direction vector of
        % attacker motion
        if(atk_pos(1)>=0)
            dummy_x = 3;
        end
        if(atk_pos(1)<0)
            dummy_x = -3;
        end 
        % conditions for attacker to choose left or right tangent path
        % opt_intr_point indicates the optimal intrusion point on protected
        % circle to which attacker is heading
        if((theta >= 0) && (theta <= 180))
            tan_slope = (tan_point2(2)-atk_pos(2))/(tan_point2(1)-atk_pos(1));
            opt_intr_point = optimal_intrusion(atk_pos, tan_point2, cir_cent, r,v);
            tangent = "left";
        end
        if((theta < 0) && (theta >= -180))
            tan_slope = (tan_point1(2)-atk_pos(2))/(tan_point1(1)-atk_pos(1));
            opt_intr_point = optimal_intrusion(atk_pos, tan_point1, cir_cent, r,v);
            tangent = "right";
        end
    end
    
    % PID controller to control defender speed once theta is near zero
    if(abs(theta)<0.3)
        error = (theta-0);
        integral = integral + error*dt;
        differ = (error - prev_error)/dt;
        def_speed = k_p*error + k_i*integral + k_d*differ;
        def_pos = r*[cos(def_speed*dt + def_ang_rad);sin(def_speed*dt + def_ang_rad)];
    end
    prev_error = (theta - 0);
    % updating defender position depending on the position of attacker
    if((theta > 0.3) && (theta <= 180))
        def_speed = max_def_speed;
        def_pos = r*[cos(def_speed*dt + def_ang_rad);sin(def_speed*dt + def_ang_rad)];
    end
    if((theta < -0.3) && (theta >= -180))
        def_speed = max_def_speed;
        def_pos = r*[cos(-def_speed*dt + def_ang_rad);sin(-def_speed*dt + def_ang_rad)];
    end
       
    % updating attacker position using parametric equation of line
    dummy_y = tan_slope * dummy_x;
    atk_dir = atan2d(dummy_y,dummy_x)+180;
    velocity = max_atk_speed*[cosd(atk_dir);sind(atk_dir)];
    atk_pos = atk_pos + velocity * dt;
    
    % plot of the animation
    plot(atk_pos(1),atk_pos(2),'s','MarkerFaceColor','blue','MarkerSize',10) % plot attacker position
    plot(def_pos(1),def_pos(2),'o','MarkerFaceColor','red','MarkerSize',8) % plot defender position
    if(tangent == "right")
        line([init_atk_pos(1),tan_point1(1)],[init_atk_pos(2),tan_point1(2)])
    end
    if(tangent == "left")
        line([init_atk_pos(1),tan_point2(1)],[init_atk_pos(2),tan_point2(2)])
    end
    line([init_atk_pos(1),atk_pos(1)],[init_atk_pos(2),atk_pos(2)],'color','magenta','LineWidth',2)
    % breaks the loop if attacker reaches optimal intrusion point
    if ((abs(atk_pos(1) - opt_intr_point(1))<0.01) && (abs(atk_pos(2) - opt_intr_point(2))<0.01))
            break;
    end
    if(t==1)
        pause(1);
    end
    drawnow
end 

% function to obtain two tangent points at v-circle from attacker position
function [Q1,Q2] = tangent_point(P1,P2,r)
     P = P1-P2;
     d2 = dot(P,P);
     Q0 = P2+r^2/d2*P;
     T = [0 1;-1 0]*(r/d2)*sqrt(d2-r^2)*P;
     Q1 = Q0+T;
     Q2 = Q0-T;
end

% function to obtain angle between attacker and defender (theta)
function theta = angle(atk_pos,def_pos)
    theta = atan2d(atk_pos(2),atk_pos(1))-atan2d(def_pos(2),def_pos(1));
    if (abs(theta) > 180)
        theta = theta - 360*sign(theta);
    end
end

% function to obtain optimal intrusion point for attacker
function opt_pnt = optimal_intrusion(atk_pos, tan_point, center, r,v)
    if(v == r)
        opt_pnt = tan_point;
    end
    p = polyfit([atk_pos(1),tan_point(1)],[atk_pos(2),tan_point(2)],1);
    [x_intrsct,y_intrsct] = linecirc(p(1),p(2),center(1),center(2),r);
    intrsct1 = [x_intrsct(1) y_intrsct(1)]';
    intrsct2 = [x_intrsct(2) y_intrsct(2)]';
    dist1 = dist_calc(atk_pos,intrsct1);
    dist2 = dist_calc(atk_pos,intrsct2);
    if(dist1<dist2)
        opt_pnt = intrsct1;
    end
    if(dist2<dist1)
        opt_pnt = intrsct2;
    end  
end

% function to calculate distance between two points
function distance = dist_calc(p1,p2)
    distance = sqrt(((p2(1)-p1(1))^2)+((p2(2)-p1(2))^2));
end

