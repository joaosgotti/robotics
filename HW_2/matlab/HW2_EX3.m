clear;clc;

syms sigma l D th2 th3 th4 ;
doff = 4;

PJ_DH = [0     sigma     0   pi/2    0
         th2       0    l   0   0
         th3     0      l    0    0
         th4      0     D    0     0];   
     
matriz_final = [0 1 0 1.5*l;
                0 0 -1 1;
                -1 0 0 1.5*l;
                0 0 0 1]

[Ti,Tf_generica] = MGD_HD(PJ_DH);
Ti = simplify(Ti);
Tf_generica = simplify(Tf_generica)


PJ_DH_n = eval(subs(PJ_DH,[sigma th2 th3 th4 l D],[1 0 0 0 4 1]));

L = DrawRobotRPR(PJ_DH_n);

Robot2 = SerialLink(L,'name','Robot2');
% Robot2.teach();


phi = atan2(matriz_final(3,1),matriz_final(1,1));

D = 1;
L = 4;

sigma_max=(1.5*L)+ D+sqrt((2*L)^2-(1.5*L)^2);
sigma_min=(1.5*L)+ D-sqrt((2*L)^2-(1.5*L)^2);

var = linspace(sigma_min,sigma_max,20);

for i = 1:20
    Z(i,:) = calcula_angulos(var(i),L,D)   
end
   
for i = 1:20
    W=[-15 15 -15 15 -15 15];
    Robot2.plot(Z(i,:),'workspace',W);
    pause(1);
end


function [M] = calcula_angulos(sigma,L,D)

        ppx = 1.5*L;
        ppz = 1.5*L + D;
        
        t3 = acos((ppx^2 + (ppz - sigma)^2 - 2*L^2)/(2*L^2));
        t2 = atan2(ppz - sigma, ppx) - atan2(L*sin(t3), L + L*cos(t3));
        t4 = -pi/2 - t2 - t3;
        
        M = [sigma,t2,t3,t4];
end

function [M,TF] = MGD_HD(tab)
M = sym(zeros(4,4,4));
for i = 1:4
    M(:,:,i) = [cos(tab(i,1) + tab(i,5)),-sin(tab(i,1) + tab(i,5))*cos(tab(i,4)),  sin(tab(i,1) + tab(i,5))*sin(tab(i,4)),tab(i,3)*cos(tab(i,1) + tab(i,5));
                sin(tab(i,1) + tab(i,5)), cos(tab(i,1) + tab(i,5))*cos(tab(i,4)), -cos(tab(i,1) + tab(i,5))*sin(tab(i,4)),tab(i,3)*sin(tab(i,1) + tab(i,5));
                0 , sin(tab(i,4)), cos(tab(i,4)), tab(i,2);
                0 , 0, 0 , 1];
end
TF =  M(:,:,1) *  M(:,:,2) *  M(:,:,3) *  M(:,:,4);
end

    function L = DrawRobotRPR(tab);
    L(1) = Link('prismatic', 'theta', double(tab(1,2)), 'a', double(tab(1,3)), 'alpha', double(tab(1,4)), 'offset', double(tab(1,5)),'qlim',[0 pi]);
    L(2) = Link('revolute'	,'d', double(tab(2,2)), 'a', double(tab(2,3)), 'alpha', double(tab(2,4)), 'offset',double(tab(2,5)),'qlim',[0 pi]);
    L(3) = Link('revolute', 'd', double(tab(3,2)), 'a', double(tab(3,3)), 'alpha', double(tab(3,4)), 'offset',double(tab(3,5)),'qlim',[0 pi]);
    L(4) = Link('revolute', 'd', double(tab(4,2)), 'a', double(tab(4,4)), 'alpha', double(tab(4,4)), 'offset',double(tab(4,5)), 'qlim', [0 0]);
    end