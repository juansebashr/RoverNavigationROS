%% Navegación autonoma y control servo visual de rover modelo Rocker-Bogie y ROS
%Juan Sebastián Hernández Reyes
%Gustavo Adolfo Berdugo Velasquez

%% Navegacion

%Obtencion del binaryOccupancyGrid
mapa=imread('mapaBN.png');
mapa=rgb2gray(mapa);
imageNorm = double(mapa)/255;
imageOccupancy = 1 - imageNorm;
map = binaryOccupancyMap(imageOccupancy,19);
map.GridLocationInWorld=[-2 -2];
figure(1)
show(map)

%Inflado y obtencion del C-Space
inflate(map,0.4)

%Comparación con el mapa sin inflar
edges=edge(imageOccupancy);
mat=occupancyMatrix(map);
tot=edges+(1-mat);
figure(2)
imshow(tot);

%% Obtención de la trayectoria

%Trayectoria por mapa probabilistico
planner=mobileRobotPRM(map);
planner.NumNodes=500;
PPRTra=findpath(planner,[0 0],[21 26]);
figure(3)
show(planner)

%Trayectoria por Voronoi
bordes=imread('bordes.png');
bordes=rgb2gray(bordes);
[y,x]=find(bordes);
[vx,vy] = voronoi(x/19,(30-y/19));

indicesx1 = find(vx>25);
vx(indicesx1) = NaN;
indicesx2 = find(vx<0);
vx(indicesx2) = NaN;
indicesy1 = find(vy>30);
vy(indicesy1) = NaN;
indicesy2 = find(vy<0);
vy(indicesy2) = NaN;    

vx=vx-2;
vy=vy-2;

map2=map;
inflate(map2,0.25);
map = binaryOccupancyMap(imageOccupancy,19);
map.GridLocationInWorld=[-2 -2];
inflate(map,0.5)

for i=1:length(vx)
    for j=1:2
    if(checkOccupancy(map2,[vx(j,i) vy(j,i)]))
        vx(j,i)=NaN;
        vy(j,i)=NaN;
    end
    end
end

vx=rmmissing(vx,2);
vy=rmmissing(vy,2);

for i=1:length(vx)
    if(fix(100*vx(1,i))==fix(100*vx(2,i)) && fix(100*vy(1,i))==fix(100*vy(2,i)))
        vx(:,i)=NaN;
        vy(:,i)=NaN;
    end
end

vx=rmmissing(vx,2);
vy=rmmissing(vy,2);

figure(4)
hold on
show(map)
plot(vxn,vyn,'-b');
hold off;

%% Matlab con ROS

rosinit('192.168.242.128',11311)

Lidar = rossubscriber('rrbot/laser/scan');

img = rossubscriber('/camera1/image_raw');

TF = rossubscriber('/odom','nav_msgs/Odometry');

%Publicadores para las ruedas izquierdas
wlf=rospublisher('/wheel_LF_controller/command','std_msgs/Float64');
wlm=rospublisher('/wheel_LM_controller/command','std_msgs/Float64');
wlb=rospublisher('/wheel_LB_controller/command','std_msgs/Float64');
%Publicadores para las ruedas derechas
wrf=rospublisher('/wheel_RF_controller/command','std_msgs/Float64');
wrm=rospublisher('/wheel_RM_controller/command','std_msgs/Float64');
wrb=rospublisher('/wheel_RB_controller/command','std_msgs/Float64');

%Se definen los handles para los mensajes del Robot
            
WLF = rosmessage(wlf);
WLM = rosmessage(wlm);
WLB = rosmessage(wlb);
WRF = rosmessage(wrf);
WRM = rosmessage(wrm);
WRB = rosmessage(wrb);
%% Estrategia de Control 2

%Control por Pure Pursuit
pp=controllerPurePursuit('MaxAngularVelocity',3,'DesiredLinearVelocity',0.5,'LookaheadDistance',0.7);
pp.Waypoints = PPRTra;

%Modelo cinemático simplificado del robot
c = 0.04;
b = 0.14;
F = [1/c b/c; 1/c -b/c];

%Conexión y envios de mensajes a MATLAB
while 1
    %Calculo de la odometria
    posx = TF.LatestMessage.Pose.Pose.Position.X;
    posy = TF.LatestMessage.Pose.Pose.Position.Y;
    cuac = [TF.LatestMessage.Pose.Pose.Orientation.X TF.LatestMessage.Pose.Pose.Orientation.Y TF.LatestMessage.Pose.Pose.Orientation.Z TF.LatestMessage.Pose.Pose.Orientation.W];
    eule = quat2eul(cuac);
    orir = eule(3);
    
    %Calculo del error de posición
        error_x = PPRTra(end,1) - posx;
    error_y = PPRTra(end,2) - posy;
    rho = sqrt(error_x^2 + error_y^2);
    
    %Criterio de terminación
    if rho < 0.1
        break
    end
    %Calculo del controlador
    [v,w] = pp([posx posy orir]);
    
    %Converisión a velocidades de las ruedas
    W = F*[v w]';

    wr = W(1);
    wl = W(2);
    
    %Envio de mensajes a ROS
    WLF.Data = wl;
    WLM.Data = wl;
    WLB.Data = wl;
    WRF.Data = wr;
    WRM.Data = wr;
    WRB.Data = wr;

    send(wlf,WLF);
    send(wlb,WLF);
    send(wlm,WLF);
    send(wrf,WRF);
    send(wrm,WRM);
    send(wrb,WRB);
    
    %Vision de máquina: Identificación de hielo
    
    hielo = readImage(img.LatestMessage);
    hieloHSV=rgb2hsv(hielo);
    minval = [0.5,0.6,0];
    maxval = [1,1,0.6];
    %Aplicación de mascara
    out = true(size(hieloHSV,1), size(hieloHSV,2));
    for p = 1 : 3
    out = out & (hieloHSV(:,:,p) >= minval(p) & hieloHSV(:,:,p) <= maxval(p));
    end
    %Ploteo de los rectángulos
    figure(4)
    imshow(hielo)
    %Identificación y etiquetado de regiones
    [L, Ne]=bwlabel(out);
    props=regionprops(L,'Centroid','Orientation','BoundingBox');
    hold on
    for n=1:Ne
        rotate(rectangle('Position',props(n).BoundingBox,"Edgecolor",[0 1 0],"LineWidth",5),[0 0 1],45);
    end
    hold off
    %Frecuencia de mensajes -> 5Hz
    pause(0.2)
end

%Envio de datos a ROS

WLF.Data = 0;
WLM.Data = 0;
WLB.Data = 0;
WRF.Data = 0;
WRM.Data = 0;
WRB.Data = 0;

send(wlf,WLF);
send(wlb,WLF);
send(wlm,WLF);
send(wrf,WRF);
send(wrm,WRM);
send(wrb,WRB);

rosshutdown

%% Estrategia de control 1: Control Polar

%Ganacias proporcionales
kp = 3;
ka = 8;
kb = -0.2;

%Indicador del goal
k = 1;

%Cinemática inversa
c = 0.04;
b = 0.14;
F = [1/c b/c; 1/c -b/c];
v_max = 0.5;
w_max = 3;

%Aplicación del algoritmo de control
while k < size(PPRTra,1)
    
    %Posiciones del robot
    posx = TF.LatestMessage.Pose.Pose.Position.X;
    posy = TF.LatestMessage.Pose.Pose.Position.Y;
    cuac = [TF.LatestMessage.Pose.Pose.Orientation.X TF.LatestMessage.Pose.Pose.Orientation.Y TF.LatestMessage.Pose.Pose.Orientation.Z TF.LatestMessage.Pose.Pose.Orientation.W];
    eule = quat2eul(cuac);
    orir = eule(3);

    %Calculo de errores
    error_x = PPRTra(k,1) - posx;
    error_y = PPRTra(k,2) - posy;

    %Parametrso de control polar 
    alpha = -orir + atan2(error_y,error_x);
    beta = -orir-alpha;
    rho = sqrt(error_x^2 + error_y^2);
    
    if alpha <= -pi  
            alpha = alpha + 2*pi;
    elseif alpha > pi
            alpha = alpha - 2*pi;
    end
    
    v = kp*rho;
    w = ka*alpha + kb*beta;

    if rho < 0.1
        k=k+1;
    end

    %Limites de velocidad
    if v > v_max
        v = v_max;
    end
    if w > w_max
        w = w_max;
    end
    if w < w_max
        w = w_max;
    end

    %Velocidades angulares de las ruedas
    W = F*[v w]';

    wr = W(1);
    wl = W(2);
end