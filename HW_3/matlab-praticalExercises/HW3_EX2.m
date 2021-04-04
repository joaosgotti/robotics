%%
%Exercicio 2 da Labwork 3
%PL 2
%Miguel Mendes Silva
%Joao Vítor Sgotti Veiga
%%
clear;clc;

syms sigma th2 th3 th4 ;
LL =2;
D = 1;


PJ_DH = [0     sigma     0   pi/2    0
         th2       0    LL   0   0
         th3     0      LL    0    0
         th4      0     D    0     0];   
     
T_espaco_tarefas = [0 1 0 LL;
                0 0 -1 1;
                -1 0 0 LL;
                0 0 0 1];

[Ti,Tf_generica] = MGD_HD(PJ_DH);
Ti = simplify(Ti);
Tf_generica = simplify(Tf_generica);

W=[-5 5 -5 5 -5 10];



L = DrawRobotRPR(PJ_DH);
Robot2 = SerialLink(L,'name','Robot2');
figure(1)
title({'Posicao default do robot','  '}) 
Robot2.plot([0 0 0 0],"workspace",W);  %robot a ser desejado na posiçao base
disp("Pressione ENTER para prosseguir");
pause();clc;



LL = 2;
D = 1;

Jac = Jacobiano(Tf_generica,sigma,th2,th3,th4);

sigma_max=(LL)+ D+sqrt((2*LL)^2-(LL)^2);
sigma_min=(LL)+ D-sqrt((2*LL)^2-(LL)^2);




parametros_iniciais = calcula_angulos(0,LL,D);
title({'Posicao com sigma minimo','  '}) 
Robot2.plot([parametros_iniciais],"workspace",W);  %robot a ser desejado com um sigma minimo!
disp("Pressione ENTER para prosseguir");
pause();clc;


parametros_finais = calcula_angulos(sigma_max,LL,D);
title({'Posicao com sigma maximo','  '}) 
Robot2.plot([parametros_finais],"workspace",W);  %robot a ser desejado com punho sigma minimo!
disp("Pressione ENTER para prosseguir");
pause();clc;
%com este close fechamos a primeira figure!
close(1) 

q_inicial = calcula_angulos(0,LL,D);

sigma_der = 5;
%pedimos os parametros ao user
[kp,deltat] = parametros;

for x = 1:200

    angulos(x,:) = q_inicial;
    px = eval(subs(Tf_generica(1,4),[sigma,th2,th3,th4],[q_inicial])); %posicao atual
    pz = eval(subs(Tf_generica(3,4),[sigma,th2,th3,th4],[q_inicial])); %posicao atual
    
    vx = (LL- px)/deltat; %vx =  posicao ideal - posiçao atual
    vz = (LL-pz)/deltat;
    wy = (-pi/2 - (q_inicial(1,2)+ q_inicial(1,3) + q_inicial(1,4)))/deltat;
    
    velocidade_x(x) = vx;
    velocidade_z(x) = vz;
    velocidade_w(x) = wy;
    
    Jacob= [Jac(1,:);Jac(3,:);Jac(5,:)];
    
    Jacob = eval(subs(Jacob,[sigma,th2,th3,th4],q_inicial));
    q_der = pinv(Jacob(:,2:4)) * [vx;vz-sigma_der;wy];
    
    q_der = [sigma_der q_der'];
    novos_q = q_inicial + q_der*deltat*kp;
    
    
    q_inicial = novos_q;
    %restiçoes para o sigma nunca ser superior a sigma_max e inferior a
    %siga_min
    %se na proxima iteraçao, se o valor de sigma for superior a sigma_max,
    %entao o programa altera a velocidade na iteraçao atual.
    if (novos_q(1,1) + (sigma_der*deltat*kp)) >sigma_max   
        sigma_der = -5;
    end
    if (novos_q(1,1) + (sigma_der*deltat*kp))<= 0 
        t = calcula_angulos(0,2,1);
        q_inicial = [0, t(:,2:4)];
        sigma_der = 5;
    end
end


%aqui fechamos a primeira Figure e abrimos a segunda

figure(2)
title({'Movimento do braço','  '}) 
Robot2.plot([angulos],"workspace",W);

%graficos velocidade
lim = (x*deltat)-deltat;


figure(3)
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



function [M] = calcula_angulos(sigma,LL,D)

        ppx = LL;
        ppz = LL + D;
        
        t3 = -acos((ppx^2 + (ppz - sigma)^2 - 2*LL^2)/(2*LL^2));
        t2 = atan2(ppz - sigma, ppx) - atan2(LL*sin(t3), LL + LL*cos(t3));
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

L(1) = Link('prismatic', 'theta', double(tab(1,1)), 'a', double(tab(1,3)), 'alpha', double(tab(1,4)), 'offset', double(tab(1,5)),'qlim',[0 pi]);
L(2) = Link('revolute'	,'d', double(tab(2,2)), 'a', double(tab(2,3)), 'alpha', double(tab(2,4)), 'offset',double(tab(2,5)),'qlim',[0 pi]);
L(3) = Link('revolute', 'd', double(tab(3,2)), 'a', double(tab(3,3)), 'alpha', double(tab(3,4)), 'offset',double(tab(3,5)),'qlim',[0 pi]);
L(4) = Link('revolute', 'd', double(tab(4,2)), 'a', double(tab(4,3)), 'alpha', double(tab(4,4)), 'offset',double(tab(4,5)), 'qlim', [0 0]);
end

function Jacob = Jacobiano(Tf,sigma,th2,th3,th4)
    OGP = Tf(:,4);
    OGP(end,:) = [];
    
    Jacob_v = simplify([diff(OGP,sigma) diff(OGP,th2) diff(OGP,th3) diff(OGP,th4)]);

    Jacob_w = [[0;0;0],[0;-1;0],[0;-1;0],[0;-1;0]];
    
    Jacob = simplify([Jacob_v;Jacob_w]);
    
end

function [var1,var2] =  parametros
        var1 = input("Introduza um kp para o sistema: ");
        var2 = input("Introduza um deltat para o sistema: ");
        disp("A processar...");
end