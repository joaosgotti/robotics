%%
%Exercicio 1 da Labwork 3  ->> abordagem integradora
%PL 2
%Miguel Mendes Silva
%Joao Vítor Sgotti Veiga

%%
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

figure(1);
H = [-10 70 -10 15 -10 40];
title({'Posicao default do robot','  '}) 
Robot.plot([0 0 5 0 0],"workspace",H);
disp("Pressione ENTER para prosseguir");
pause();
clc;

%pedimos aqui os parametros ao user
[r,deltat] = parametros;

T_tarefas_numerica = eval(subs(T_tarefas,[alpha,r],[0 0]));

[t0 t1 d t3] = inversa(T_tarefas_numerica,0,0);
q = [t0,t1,d,t3];

close(1);
figure(2);
title({'Posicao inicial do robot','  '}) 
Robot.plot([q 0],"workspace",H)
disp("Pressione ENTER para prosseguir");
pause();clc;
disp("A calcular...");


Jacob = Jacobiano(Tf_generica,Ti,th0,th1,d2,th3);

alpha = 0;
Dalpha = pi/2;
v = [-r*sin(alpha)*Dalpha;0;-r*cos(alpha)*Dalpha];
w = [0;Dalpha;0];
velo = [v;w];

for i= 1:1:200
   
    angulos(i,:) = q;
    
    Jac  = eval(subs(Jacob,[th0 th1 d2 th3],q));
    v = [-r*sin(alpha)*Dalpha;0;-r*cos(alpha)*Dalpha];
    w = [0;Dalpha;0];
    velo = [v;w];
    
    q_der = pinv(Jac) * velo; %matriz fica 4x1
    
    novos_q = q + q_der'*deltat;
    velocidade_x(i) = velo(1,1);
    velocidade_z(i) = velo(3,1);
    velocidade_w(i) = velo(5,1);
    
    %q = novos_q;
    
    %[t0 t1 d t3] = inversa(T_tarefas_numerica,0,alpha);
    %q = [t0,t1,d,t3];
    
    
    alpha = alpha + Dalpha*deltat;
    q = novos_q;

end
close(2);
clc;angulos(:,end+1) = [0];

figure(3)
title({'Movimento do braço da abordagem integradora','  '}) 
Robot.plot([angulos],"workspace",H);

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
    r = r_value;
    alpha = alpha_value;
    
    t0 = atan2(T_gen_numerica(2,4),T_gen_numerica(1,4));
    t1 = atan2(20-(r-10)*sin(alpha),40+(r-10)*cos(alpha));
    dd = cos(t1)*(40 +(r-10)*cos(alpha)) + (sin(t1)*(20 - (r -10)* sin(alpha)));
    t3 = -alpha - t1;
  
end

function Jacob = Jacobiano(Tf,Ti,th0,th1,d2,th3)
    OGP = simplify([Tf(1,4);Tf(2,4);Tf(3,4)]);
    
 
    Jacob_v = simplify([diff(OGP,th0) diff(OGP,th1) diff(OGP,d2) diff(OGP,th3)]);
    
    O3R = Ti(:,:,1)*Ti(:,:,2)*Ti(:,:,3);
    O3R= O3R(:,3);

    Jacob_w = [[0;0;1;0],Ti(:,3,1),[0;0;0;0],O3R];
    Jacob_w(end,:)=[];

    Jacob = simplify([Jacob_v;Jacob_w]);
    
end

function [var1,var2] =  parametros
        var1 = input("Introduza um raio para o sistema: ");
        var2 = input("Introduza um deltat para o sistema: ");
        clc;
end


