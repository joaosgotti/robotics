%%
%Exercicio 1 da Labwork 3  ->> controlo de malha fechada
%PL 2
%Miguel Mendes Silva
%Joao Vítor Sgotti Veiga

%%
%valores ideais de KP= 0.5;
%valores ideais de deltat = 0.1;

clear;clc;

doff = 2;

syms th0 th1 d2 th3 ;
syms alpha r;
syms Dalpha;
tab = [th0 th1 d2 th3];

PJ_DH = [th0 0   0   pi/2    0
         th1 0   0   pi/2    pi/2
         0   d2  0   -pi/2  0
         th3 0   0   pi/2    0
         0   10  0     0   -pi/2];
     

T_tarefas = [0 sin(alpha) cos(alpha) 40+r*cos(alpha)
             1  0          0          0
             0 cos(alpha) -sin(alpha) 20-r*cos(alpha)
             0 0          0           1];
            
              
         
[Ti,Tf_generica] = MGD_HD(PJ_DH);
Ti  =simplify(Ti);
Tf_generica = simplify(Tf_generica);

L = DrawRobotRRR(PJ_DH);
Robot = SerialLink(L,'name','Robot');

figure(1)
title({'Posiçao default do robot','  '}) 
H = [-10 70 -10 15 -10 40];
Robot.plot([0 0 5 0 0],"workspace",H);
disp("Pressione ENTER para prosseguir");
pause();
clc;close(1);
%parametros

[r,kp,deltat] = parametros;

T_tarefas_numerica = eval(subs(T_tarefas,[alpha,r],[0 r]));

[t0 t1 d t3] = inversa(T_tarefas_numerica,r,0);
q = [t0,t1,d,t3];

figure(2)
title({'Posiçao inicial do robot','  '}) 
Robot.plot([q 0],"workspace",H)
pause();
disp("A processar...");


Jacob = Jacobiano(Tf_generica,Ti,th0,th1,d2,th3);
Jac  = eval(subs(Jacob,[th0 th1 d2 th3],q));


Dalpha = pi/2;
alpha = 0;

for i= 1:1:100
    angulos(i,:) = q;
    
    vx = -r*sin(alpha)*Dalpha;
    vz = -r*cos(alpha)*Dalpha;
    wy = pi/2;
    
    px = eval(subs(Tf_generica(1,4),[th0 th1 d2 th3],q));  %posicao atual
    pz = eval(subs(Tf_generica(3,4),[th0 th1 d2 th3],q));  
    
    x = (40 + r*cos(alpha)) - r*sin(alpha)*Dalpha*deltat; %desejada pertence à circunferencia
    z = (20 - r*sin(alpha)) - r*cos(alpha)*Dalpha*deltat;
    
    alpha = alpha + Dalpha*deltat;
    wy = ((alpha - (-q(1,2) - q(1,4)))/deltat) +  wy;
    
    vx = (x-px)/deltat + vx;   
    vz = (z-pz)/deltat + vz;
    
    velocidade_x(i) = vx;
    velocidade_z(i) = vz;
    velocidade_w(i) = wy;
    
    Jac  = eval(subs(Jacob,[th0 th1 d2 th3],q));
    q_der = pinv(Jac) * [vx;0;vz;0;wy;0];
   
    novos_q = q +q_der'*deltat*kp; %angulos da proxima iteraçao
   
    q = novos_q;  %update
end
%%
close(2);
angulos(:,end+1) = [0];

figure(3)
title({'Movimento do braço na abordagem de malha fechada','  '}) 
Robot.plot([angulos],"workspace",H)    


lim = (i*deltat)-deltat;


figure(4)
plot([0:deltat:lim],velocidade_x,'red','DisplayName','Vx');
hold on
plot([0:deltat:lim],velocidade_z,'blue','DisplayName','Vz');
hold on
plot([0:deltat:lim],velocidade_w,'green','DisplayName','Wy');
hold on
title("Graficos Velocidade");
xlabel("tempo (s)");
ylabel("velocidades");
legend 

function L = DrawRobotRRR(tab)

    L(1) = Link('revolute', 'd', double(tab(1,2)), 'a', double(tab(1,3)), 'alpha', double(tab(1,4)), 'offset', double(tab(1,5)),'qlim',[0 pi]);
    L(2) = Link('revolute', 'd', double(tab(2,2)), 'a', double(tab(2,3)), 'alpha', double(tab(2,4)), 'offset',double(tab(2,5)),'qlim',[0 pi]);
    L(3) = Link('prismatic', 'theta', double(tab(3,1)), 'a', double(tab(3,3)), 'alpha', double(tab(3,4)), 'offset',double(tab(3,5)),'qlim',[0 2*pi]);
    L(4) = Link('revolute', 'd', double(tab(4,2)), 'a', double(tab(4,3)), 'alpha', double(tab(4,4)), 'offset',double(tab(4,5)), 'qlim', [0 pi]);
    L(5) = Link('revolute', 'd', double(tab(5,2)), 'a', double(tab(5,3)), 'alpha', double(tab(5,4)), 'offset',double(tab(5,5)), 'qlim', [0 pi]);
end

function [t0,t1,dd,t3] = inversa(T_gen_numerica,r_value,alpha_value)
    r =r_value;
    alpha = alpha_value;
    
    t0 = atan2(T_gen_numerica(2,4),T_gen_numerica(1,4));
    t1 = atan2(20-(r-10)*sin(alpha),40+(r-10)*cos(alpha));
    dd = cos(t1)*(40 +(r-10)*cos(alpha)) + (sin(t1)*(20 - (r -10)* sin(alpha)));
    t3 = -alpha - t1;
  
end

function Jacob = Jacobiano(Tf,Ti,th0,th1,d2,th3)
   
    OGP = simplify([Tf(1,4);Tf(2,4);Tf(3,4)]);
    
    Jacob_v = simplify([diff(OGP,th0) diff(OGP,th1) diff(OGP,d2) diff(OGP,th3)]);
    
    OOR = eye(3);
    O1R = Ti(:,:,1);
    O2R = Ti(:,:,1)*Ti(:,:,2)
    O3R = Ti(:,:,1)*Ti(:,:,2)*Ti(:,:,3);
    
    OOR = OOR(1:3,3);
    O1R = O1R(1:3,3);
    O2R = O2R(1:3,3);
    O3R = O3R(1:3,3);

    Jacob_w = [OOR,O1R,O2R,O3R];
   

    Jacob = simplify([Jacob_v;Jacob_w])
    
end


function [var1,var2,var3] =  parametros
        var1 = input("Introduza um raio para o sistema: ");
        var2 = input("Introduza um kp para o sistema: ");
        var3 = input("Introduza um deltat para o sistema: ");
        disp("Carregue ENTER para proseguir");
end