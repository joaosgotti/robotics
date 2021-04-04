%==========================================%
% Robótica                                 %
% Homework 1                               %
% João Vítor Sgotti Veiga (nº 2017170653)  %
% Miguel Silva (nº 2017257766)             %
%==========================================%

clc;
clear all;
view(3)
%---------------------- DEFINIÇÕES -----------------------%
p1 = [0 0 0];
p2 = [2 0 0];
p3 = [0 3 0];
p4 = [2 3 0];
p5 = [0 0 4];
p6 = [2 0 4];
p7 = [0 3 4];
p8 = [2 3 4];

obj = [p1(1) p2(1) p3(1) p4(1) p5(1) p6(1) p7(1) p8(1)
       p1(2) p2(2) p3(2) p4(2) p5(2) p6(2) p7(2) p8(2)
       p1(3) p2(3) p3(3) p4(3) p5(3) p6(3) p7(3) p8(3)
        1     1     1     1    1      1     1     1  ];

W = [+1 +0 +0 +3  %sistema de cordenada WORLD
     +0 +0 -1 -3
     +0 +1 +0 +5
     +0 +0 +0 +1];
%================================================
% desenha objeto no eixo cartesiano
I=[1 0 0 0;
    0 1 0 0;
    0 0 1 0;
    0 0 0 1];

axis([-15 15 -15 15 -15 15]); %define limites do plot
grid on; 
xlabel('Eixo XX')
ylabel('Eixo YY')
zlabel('Eixo ZZ')

v = "Cenas"
line(1.5*xlim, [0,0], [0,0], 'LineWidth', 1, 'Color', 'k','DisplayName',v);
line([0,0], 1.5*ylim, [0,0], 'LineWidth', 1, 'Color', 'k');
line([0,0], [0,0], 1.5*zlim, 'LineWidth', 1, 'Color', 'k');

trplot(I,'length',20,'color','k');
%================================================
% levando o objeto ao mundo

h = fill3(Xobj,Yobj,Zobj,'r')

obj2 = W*obj;         
[Xobj,Yobj,Zobj] = desenha_obj(obj2);
teste(obj2,h);
hold on;
trplot(W,'length',10,'color','m','mundo')

pause();
%===============================================
% primeira rotação
% phi = pi/4;

for theta=0: 0.05 : pi/4
    R1 = [1          0           0            0
          0       cos(theta)   -sin(theta)        0
          0       sin(theta)    cos(theta)        0
          0          0           0            1];
    
    obj3 = R1*W*obj;   %rotaçao R1 aplicada  como Pré-Multiplicaçao ào (WorldxCubo) --> implica rotaçao sobre eixo referencial base, que neste caso é o World
    teste(obj3,h);
    pause(0.09);
end
pause()

%=============================================
%deslocação em 5 unidades

for z=0:0.5:5
    D = [1 0 0 0
         0 1 0 0
         0 0 1 z
         0 0 0 1]
          
     obj4 = R1*W*D*obj     % Matriz D multiplica depois de R1*W porque estamos a falar do eixo proprio do cubo e nao do refeencial
     teste(obj4,h)
 
     pause(0.009)
end
pause()
%===========================================
% Rotacao eixo arbitrário 
rx=-1;
ry=1;
rz=1;
r=[rx ry rz];
rnorm=norm(r);
r=r./rnorm;
rx=r(1);
ry=r(2);
rz=r(3)

for phi = 0: -0.05: -pi/3
   Vphi=1-cos(phi);
    
   R2= [rx^2*Vphi+cos(phi)       rx*ry*Vphi-rz*sin(phi)     rx*rz*Vphi+ry*sin(phi)     0 
      rx*ry*Vphi+rz*sin(phi)   ry^2*Vphi+cos(phi)         ry*rz*Vphi-rx*sin(phi)     0
      rx*rz*Vphi-ry*sin(phi)   ry*rz*Vphi+rx*sin(phi)     rz^2*Vphi+cos(phi)         0
      0                        0                          0                          1];

     obj5 = W*R2*inv(W)*R1*W*D*obj
     %obj5 = R1*W*R2*D*obj;   %R2 multiplica depois das transformaçoes ja feitas devido a rotaçao sobre o OBJECT e nao do world
     teste(obj5,h)

     pause(0.009)
end
pause()

%===============================================
% terceira rotação
% theta = -pi/2;

for theta=0: -0.05 : -pi/2
    R3 = [cos(theta)   -sin(theta)        0 0
          sin(theta)    cos(theta)        0 0
          0             0                 1 0
          0             0                 0 1];
    
    obj6 = inv(R3)*W*R2*inv(W)*R1*W*D*obj
    teste(obj6,h)
    pause(0.009);
end
pause()

%=============================================

function [Xobj,Yobj,Zobj] = desenha_obj(x)

    axis([-20 20 -20 20 -20 20])
    hold on;
    grid on;
    xlabel('x'); ylabel('y'); zlabel('z');
    
    face_frente = transpose([x(:,2) x(:,6) x(:,8) x(:,4)]);
      face_Lesq = transpose([x(:,2) x(:,6) x(:,5) x(:,1)]);
      face_Ldir = transpose([x(:,8) x(:,4) x(:,3) x(:,7)]);
      face_tras = transpose([x(:,3) x(:,7) x(:,5) x(:,1)]);
      face_cima = transpose([x(:,5) x(:,6) x(:,8) x(:,7)]);
    face_baixo  = transpose([x(:,2) x(:,4) x(:,3) x(:,1)]);
    
    
    Xobj= [face_frente(:,1) face_Lesq(:,1) face_Ldir(:,1) face_tras(:,1) face_cima(:,1) face_baixo(:,1)];
    Yobj= [face_frente(:,2) face_Lesq(:,2) face_Ldir(:,2) face_tras(:,2) face_cima(:,2) face_baixo(:,2)];
    Zobj= [face_frente(:,3) face_Lesq(:,3) face_Ldir(:,3) face_tras(:,3) face_cima(:,3) face_baixo(:,3)];
 
        
    
end

function teste(matriz,h)

    axis([-20 20 -20 20 -20 20])
    hold on;
    grid on;
    xlabel('x'); ylabel('y'); zlabel('z');
    
      face_frente = transpose([matriz(:,2) matriz(:,6) matriz(:,8) matriz(:,4)]); 
      face_Lesq = transpose([matriz(:,2) matriz(:,6) matriz(:,5) matriz(:,1)]);
      face_Ldir = transpose([matriz(:,8) matriz(:,4) matriz(:,3) matriz(:,7)]);
      face_tras = transpose([matriz(:,3) matriz(:,7) matriz(:,5) matriz(:,1)]);
      face_cima = transpose([matriz(:,5) matriz(:,6) matriz(:,8) matriz(:,7)]);
      face_baixo  = transpose([matriz(:,2) matriz(:,4) matriz(:,3) matriz(:,1)]);
       
     
      h(1).Vertices = [face_frente(:,1) face_frente(:,2) face_frente(:,3)]
      h(2).Vertices = [face_Lesq(:,1) face_Lesq(:,2) face_Lesq(:,3)]
      h(3).Vertices = [face_Ldir(:,1) face_Ldir(:,2) face_Ldir(:,3)]
      h(4).Vertices = [face_tras(:,1) face_tras(:,2) face_tras(:,3)]
      h(5).Vertices = [face_cima(:,1) face_cima(:,2) face_cima(:,3)]
      h(6).Vertices = [face_baixo(:,1) face_baixo(:,2) face_baixo(:,3)]
end





