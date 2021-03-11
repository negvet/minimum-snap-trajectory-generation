function ThreeD_interactive_yaw
% initials
x0 = 0; xd0 = 0; xdd0 = 0; xf = 2; xdf = 0;
y0 = 0; yd0 = 0; ydd0 = 0; yf = 2; ydf = 0;
z0 = 0; zd0 = 0; zdd0 = 0; zf = 2; zdf = 0;
if xf>0         yaw0=atan(yf/xf); end;
if (xf<0&&yf>0) yaw0=atan(yf/xf)+pi; end;
if (xf<0&&yf<0) yaw0=atan(yf/xf)-pi; end;
yaw_prev=yaw0; yawd0=0; yawf=0; yawdf=0;
va_g=3;        %global average velocity
t_f=sqrt(xf^2+yf^2+zf^2)/va_g; %global time to goal

f=figure('Name',       'Tag me!',...
                     'KeyPressFcn', @MyKeyDown, ...
                     'KeyReleaseFcn', @MyKeyUp); 
clf; hold on; grid on; xlabel('x'); ylabel('y'); zlabel('z'); view(3);


KeyStatus = false(1,7);
KeyNames = {'uparrow','leftarrow','rightarrow','downarrow','w','s','escape'};
KEY.UP = 1; KEY.LEFT = 2; KEY.RIGHT = 3; KEY.DOWN = 4; 
KEY.ZUP = 5; KEY.ZDOWN = 6; KEY.ESC = 7;

% set up experiment
position_x=0; position_y=0; position_z=0;
minDistGoal = .1; nearGoal=false; time=0; complete=false; 
delta_xg=0; delta_yg=0; delta_zg=0;

X0=[0;0;0;0;0;0];

r=.1; phi=linspace(0,pi,30); theta=linspace(0,2*pi,40); [phi,theta]=meshgrid(phi,theta);
x=r*sin(phi).*cos(theta); y=r*sin(phi).*sin(theta); z=r*cos(phi); 
%initial tagger position plotting
tagger=mesh(x+x0, y+y0, z+z0);
%On board 'x' axes (yaw angle)
x_yaw=-0.3:0.01:0.3; y_yaw=zeros(size(x_yaw)); z_yaw=zeros(size(x_yaw));
yaw_line=plot3(position_x+x_yaw,position_y+y_yaw,position_z+z_yaw,'LineWidth',2);
% goal position ploting
goal=mesh(x+xf, y+yf, z+zf);

while (~nearGoal&&~complete)
coefs_x = flipud(qp_xyz(x0, xd0, xdd0, xf, xdf, t_f));
coefs_y = flipud(qp_xyz(y0, yd0, ydd0, yf, ydf, t_f));
coefs_z = flipud(qp_xyz(z0, zd0, zdd0, zf, zdf, t_f));
coefs_yaw = flipud(qp_yaw(yaw0, yawd0, yawf, yawdf, t_f));
ts=.05;
% EXIT CONDITION 1 (hit the goal, for non-moving goal)
if (t_f<ts)
    ts=t_f;
    nearGoal=true;
end
% TRAJECTORY PLOTTING
interv = linspace(0, ts, 10);
plot3(polyval(coefs_x, interv)+position_x, polyval(coefs_y, interv)+position_y, polyval(coefs_z, interv)+position_z,'LineWidth',2);
delete(tagger);
tagger=mesh(x+polyval(coefs_x, ts)+position_x, y+polyval(coefs_y, ts)+position_y, z+polyval(coefs_z, ts)+position_z);
%for the next plotting, absolut values:
time = time + ts;
position_x  = position_x + polyval(coefs_x, ts);
position_y  = position_y + polyval(coefs_y, ts);
position_z  = position_z + polyval(coefs_z, ts);
%GOAL MOVING(while upper trajectory execution)
[Xout,delta_xg,delta_yg,delta_zg,complete] = goal_behaviour(ts,X0,KeyStatus,KEY);
X0=Xout(:,end);
%INITIALS FOR THE NEXT QP
dcoefs_x = polyder(coefs_x); xd0 = polyval(dcoefs_x, ts);
ddcoefs_x = polyder(dcoefs_x); xdd0 = polyval(ddcoefs_x, ts);
dcoefs_y = polyder(coefs_y); yd0 = polyval(dcoefs_y, ts);
ddcoefs_y = polyder(dcoefs_y); ydd0 = polyval(ddcoefs_y, ts);
dcoefs_z = polyder(coefs_z); zd0 = polyval(dcoefs_z, ts);
ddcoefs_z = polyder(dcoefs_z); zdd0 = polyval(ddcoefs_z, ts);
dcoefs_yaw = polyder(coefs_yaw); yawd0 = polyval(dcoefs_yaw, ts);
%xf=xf-polyval(coefs, ts); %non-moving goal
xf=xf-polyval(coefs_x, ts)+delta_xg; %moving goal
yf=yf-polyval(coefs_y, ts)+delta_yg; %moving goal
zf=zf-polyval(coefs_z, ts)+delta_zg; %moving goal

if xf>0         yaw_n=atan((yf)/(xf)); end;
if (xf<0&&yf>0) yaw_n=atan((yf)/(xf))+pi; end;
if (xf<0&&yf<0) yaw_n=atan((yf)/(xf))-pi; end;
delta_yaw=yaw_n-yaw_prev;
if abs(delta_yaw)>6 delta_yaw=yaw_n+yaw_prev; end;
yaw0 = polyval(coefs_yaw, ts)+delta_yaw;
yaw_prev=yaw_n;

%t_f=t_f-ts;
%t_f=xf/va;
%va=0.6*xf^0.5; t_f=xf/va; %time varying speed 1
va=(va_g-1)*(1-exp(-1*sqrt(xf^2+yf^2+zf^2)))+1; 
t_f=sqrt(xf^2+yf^2+zf^2)/va;

%EXIT CONDITION 2 (approach the goal, for moving goal)
if sqrt(xf^2+yf^2+zf^2)<minDistGoal
   nearGoal=true;
end

%goal plotting
delete(goal);
%goal=plot3(xf+position_x, yf+position_y, zf+position_z, 'o-r','LineWidth',2,'MarkerSize',15);
goal=mesh(x+xf+position_x, y+yf+position_y, z+zf+position_z);

%yaw line plotting
delete(yaw_line);
yaw_line=plot3(position_x+x_yaw,position_y+y_yaw,position_z+z_yaw,'-r','LineWidth',2);
rotate(yaw_line,[0 0 1],(yaw_prev-yaw0)*57.3,[position_x,position_y,0])

pause(ts);
end


%nested callbacks
function   MyKeyDown(hObject,event, handles)
            key = get(hObject,'CurrentKey');
            KeyStatus = (strcmp(key, KeyNames) | KeyStatus);
end
function  MyKeyUp(hObject,event, handles)
            key = get(hObject,'CurrentKey');
            KeyStatus = (~strcmp(key, KeyNames) & KeyStatus);
end

end %traj_interactive (main)



function [Xout,delta_xg,delta_yg,delta_zg,complete] = goal_behaviour(ts,X0,KeyStatus,KEY)

     thrust=5; 
     complete=false; %u(1)=0; u(2)=0; u(3)=0;
     u=zeros(1,3);
     if KeyStatus(KEY.UP) 
         u(2)=thrust; 
     end;
     if KeyStatus(KEY.DOWN) 
         u(2)=-thrust;
     end;
     if KeyStatus(KEY.UP)&&KeyStatus(KEY.DOWN)
         u(2)=0;
     end
     
     if KeyStatus(KEY.LEFT) 
         u(1)=-thrust; 
     end;
     if KeyStatus(KEY.RIGHT) 
         u(1)=thrust;
     end;
     if KeyStatus(KEY.LEFT)&&KeyStatus(KEY.RIGHT)
         u(1)=0;
     end
     
     if KeyStatus(KEY.ZUP) 
         u(3)=thrust; 
     end;
     if KeyStatus(KEY.ZDOWN) 
         u(3)=-thrust;
     end;
     if KeyStatus(KEY.ZUP)&&KeyStatus(KEY.ZDOWN)
         u(3)=0;
     end
     
     
     if KeyStatus(KEY.ESC)
         complete=true;
     end;
    
    tspan=[0 ts];
    Xout=goal_simulation(X0,tspan,u);
    delta_xg=Xout(1,end)-X0(1);
    delta_yg=Xout(2,end)-X0(2);
    delta_zg=Xout(3,end)-X0(3);

end %goal behaviour

%dynamics
function Xout = goal_simulation(X0,tspan,u)
m=1; k=1;
f = @(t,X)dynamics(t,X,m,u,k);
opts = odeset('AbsTol',1e-8,'RelTol',1e-8);
sol = ode45(f, tspan, X0,opts);
Xout=sol.y;
end

function dX= dynamics(t,X,m,u,k)
%X=[x;y;z;dx;dy;dz];
dx=X(4:6);
ddx=[u(1)/m-k*sign(dx(1))*dx(1)^2/m; u(2)/m-k*sign(dx(2))*dx(2)^2/m; u(3)/m-k*sign(dx(3))*dx(3)^2/m];
dX=[dx;ddx];
end









function coefs = qp_xyz(x0, xd0, xdd0, xf, xdf, t_f)

%objective
H=[0 0 0 0          0                     0
   0 0 0 0          0                     0
   0 0 0 0          0                     0
   0 0 0 0          0                     0
   0 0 0 0     2*24^2*t_f         24*120*t_f^2
   0 0 0 0    24*120*t_f^2    2*(1/3)*120^2*t_f^3];
f = zeros(6, 1);

Aeq=[1 0   0     0       0       0         % t=0      x=x0
     0 1   0     0       0       0         % t=0      xd=xd0
     0 0   2     0       0       0         % t=0      xdd=xdd0
     1 t_f t_f^2 t_f^3   t_f^4   t_f^5     % t=t_f    x=xf
     0 1   2*t_f 3*t_f^2 4*t_f^3 5*t_f^4]; % t=t_f    xd=xdf
beq= [x0 xd0 xdd0 xf xdf];

coefs = quadprog(H,f,[],[],Aeq,beq);
end


function coefs = qp_yaw(yaw0, yawd0, yawf, yawdf, t_f)

%objective
H=[0 0          0          0
   0 0          0          0
   0 0     2*4*t_f      12*t_f^2
   0 0    12*t_f^2     2*12*t_f^3];
f = zeros(4, 1);

Aeq=[1 0   0     0            % t=0      yaw=yaw0
     0 1   0     0            % t=0      yawd=yawd0
     1 t_f t_f^2 t_f^3        % t=t_f    yaw=yawf
     0 1   2*t_f 3*t_f^2];    % t=t_f    yawd=yawdf
beq= [yaw0 yawd0 yawf yawdf];

coefs = quadprog(H,f,[],[],Aeq,beq);

end

