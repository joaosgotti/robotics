 clear;clc;
syms th1 th2 a l;
a=1;l=1;

PJ_DH = [th1 0   0   -pi/2    0
         th2 a   0   pi/2    pi/2
         0   l   0   0       0  ];

[Ti_generica,Tf_generica] = MGD_HD(PJ_DH);

L = DrawRobotRR(PJ_DH);
Robot = SerialLink(L,'name','Robot');

P_carte = [(1+sqrt(2))/2, (-1+sqrt(2))/2, sqrt(2)/2     %posicoes que o end-effect vai ocupar
            sqrt(2)/2,    1,              -sqrt(2)/2
            -1,           sqrt(2)/2,      sqrt(2)/2];

[P_juntaA_t1,P_juntaA_t2] = inversa(P_carte(1,:));
[P_juntaB_t1,P_juntaB_t2] = inversa(P_carte(2,:));
[P_juntaC_t1,P_juntaC_t2] = inversa(P_carte(3,:));

P_junta = [P_juntaA_t1,P_juntaA_t2; %sequencias das posicoes (juntas)
           P_juntaB_t1,P_juntaB_t2;
           P_juntaC_t1,P_juntaC_t2;
           P_juntaA_t1,P_juntaA_t2]

%% alinea (A)
legenda = ["Posição A", "Posição B", "Posição C"]; 
for i=1:3
    str = sprintf('Posição da junta %.g',i)
    disp(P_junta(i,:));
    title(legenda(i));
    %%suptitle('Alinea A');
    Robot.plot([P_junta(i,:), 0])
    pause();
end

%% Alinea B
disp('Prima qualquer tecla para alinea B')
pause();
disp('Alinea B em funcionamento...')
title('A --> B --> C')       
%%suptitle('     Alinea B')
posicoes_J1 = P_junta(:,1);
posicoes_J2 = P_junta(:,2);

tempos = [0,3,10,13];

velocidades_J1 = velocidade(posicoes_J1,tempos);
velocidades_J2 = velocidade(posicoes_J2,tempos);

for i=1:2 
    tempo_init = tempos(i);
    tempo_final = tempos(i+1);
    duracao = tempo_final - tempo_init;
    P_junta_init = P_junta(i,:);
    P_junta_final = P_junta(i+1,:);
    vel_init = [velocidades_J1(i),velocidades_J2(i)];
    vel_final = [velocidades_J1(i+1),velocidades_J2(i+1)];
    for tempo_instantaneo = tempo_init : duracao/20 : tempo_final
      
        P_junta_1_instantanea = angulo(tempo_instantaneo,tempo_init,tempo_final,P_junta_init(1),P_junta_final(1),vel_init(1),vel_final(1));
        P_junta_2_instantanea = angulo(tempo_instantaneo,tempo_init,tempo_final,P_junta_init(2),P_junta_final(2),vel_init(2),vel_final(2));

        Robot.plot([P_junta_1_instantanea, P_junta_2_instantanea, 0]);
      
    end
end

%% Alinea C
disp('Prima qualquer tecla para alinea C')
pause();
disp('Alinea C em funcionamento...')
title('A --> B --> C --> A')  
%suptitle('     Alinea C')
posicoes_J1 = P_junta(:,1);
posicoes_J2 = P_junta(:,2);

velocidades_J1 = velocidade(posicoes_J1,tempos);
velocidades_J2 = velocidade(posicoes_J2,tempos);

velocidades_J1 =[velocidades_J1 0];
velocidades_J2 =[velocidades_J2 0];

k= 1;
h = 1;
while k <= 4
    for i=1:3
    % A --> B --> C
        tempo_init = tempos(i); %3 
        tempo_final = tempos(i+1); %10
        duracao = tempo_final - tempo_init;
        P_junta_init = P_junta(i,:);
        P_junta_final = P_junta(i+1,:);
  
        for tempo_instantaneo = tempo_init+0.1 : duracao/20 : tempo_final
            
            P_junta_1_instantanea = angulo(tempo_instantaneo,tempo_init,tempo_final,P_junta_init(1),P_junta_final(1),velocidades_J1(i),velocidades_J1(i+1));
            P_junta_2_instantanea = angulo(tempo_instantaneo,tempo_init,tempo_final,P_junta_init(2),P_junta_final(2),velocidades_J2(i),velocidades_J2(i+1));

            %Robot.plot([P_junta_1_instantanea, P_junta_2_instantanea, 0]);
            angulo1(h) = P_junta_1_instantanea;
            angulo2(h) = P_junta_2_instantanea;
            graf_tempo(h) = tempo_instantaneo;
            pontos(h,:) = [P_junta_1_instantanea P_junta_2_instantanea 0];
            h = h +1;
            
        end
        
    end
    k = k+1;
end
close(1);
figure(2)
Robot.plot(pontos);
close(2);

figure(3)
intervalo = linspace(1,h-1,h-1);
plot(intervalo, pontos(:,1),'red','Displayname','Theta_1');
hold on
plot(intervalo, pontos(:,2),'blue','Displayname','Theta_2');
hold on
plot(intervalo, pontos(:,3),'green')
hold on

%% Alínea (D)
disp('Prima qualquer tecla para alinea D')
pause();
disp('Alinea D em funcionamento...')

at_max = deg2rad(60); % Aceleração máxima de cada junt
% 1ª parte
tmA_1 = 3 - sqrt(3^2 - (2*(P_juntaB_t1 - P_juntaA_t1))/(sign(P_juntaB_t1-P_juntaA_t1)*at_max));
tmA_2 = 3 - sqrt(3^2 - (2*(P_juntaB_t2 - P_juntaA_t2))/(sign(P_juntaB_t2-P_juntaA_t2)*at_max));
% 3ª parte
tmC_1 = 7 - sqrt(7^2 + (2*(P_juntaC_t1 - P_juntaB_t1))/(sign(P_juntaB_t1-P_juntaC_t1)*at_max));
tmC_2 = 7 - sqrt(7^2 + (2*(P_juntaC_t2 - P_juntaB_t2))/(sign(P_juntaB_t2-P_juntaC_t2)*at_max));

vAB_1 = (P_juntaB_t1 - P_juntaA_t1)/(3-tmA_1/2);    % Velocidade da junta 1 entre A e B
vAB_2 = (P_juntaB_t2 - P_juntaA_t2)/(3-tmA_2/2);    % Velocidade da junta 2 entre A e B

vBC_1 = (P_juntaC_t1 - P_juntaB_t1)/(7-tmC_1/2);    % Velocidade da junta 1 entre B e C
vBC_2 = (P_juntaC_t2 - P_juntaB_t2)/(7-tmC_2/2);    % Velocidade da junta 2 entre B e C
 
% 2ª parte
tmB_1 = (vBC_1 - vAB_1)/(sign(vBC_1-vAB_1)*at_max);
tmB_2 = (vBC_2 - vAB_2)/(sign(vBC_2-vAB_2)*at_max);



% Tempos
tm_1 = [tmA_1 tmB_1 tmC_1]; 
tm_2 = [tmA_2 tmB_2 tmC_2]; 

% Velocidades
vABC_1 = [vAB_1 vBC_1];
vABC_2 = [vAB_2 vBC_2];

P1 = [P_juntaA_t1, P_juntaB_t1 ,P_juntaC_t1];
P2 = [P_juntaA_t2, P_juntaB_t2 ,P_juntaC_t2];

teta_1 = parabolica(P1,tm_1,vABC_1,at_max);
teta_2 = parabolica(P2,tm_2,vABC_2,at_max);

for i=1:101
    Thetas(i,:) = [teta_1(1,i) teta_2(1,i) 0];
    Robot.plot(Thetas(i,:))
end


function [ang] = angulo(t,t0,tf,th0,thf,vi,vf)

delta_t = tf-t0;
ang = th0 + vi*(t-t0) + ( (3/(delta_t^2))*(thf-th0) - (2/delta_t)*vi-(1/delta_t)*vf )*((t-t0)^2) - ( (2/(delta_t^3))*(thf-th0) - (1/(delta_t^2))*(vf+vi) )*((t-t0)^3);

end

function [t1,t2] = inversa(posicao_gripper)
    tx=posicao_gripper(1);    
    ty=posicao_gripper(2);
    tz=posicao_gripper(3);
    t2 = atan2(-tz,sqrt(tx^2 + ty^2 - 1));
    t1 = atan2(-tx,ty)+atan(cos(t2));
end

function L = DrawRobotRR(tab)
    L(1) = Link('revolute', 'd', double(tab(1,2)), 'a', double(tab(1,3)), 'alpha', double(tab(1,4)), 'offset', double(tab(1,5)),'qlim',[0 2*pi]);
    L(2) = Link('revolute', 'd', double(tab(2,2)), 'a', double(tab(2,3)), 'alpha', double(tab(2,4)), 'offset',double(tab(2,5)),'qlim',[0 2*pi]);
    L(3) = Link('revolute', 'd', double(tab(3,2)), 'a', double(tab(3,3)), 'alpha', double(tab(3,4)), 'offset',double(tab(3,5)),'qlim',[0 0]);
end

function [vel] = velocidade(posicao,tempo)
    
    s = size(posicao);
    s=s(1);
    vel(1)=0;
    
    for i=2:s-1
        vel_dir = (posicao(i+1)-posicao(i))/(tempo(i+1) - tempo(i));
        vel_esq = (posicao(i)-posicao(i-1))/(tempo(i) - tempo(i-1));
        
        if (sign(vel_esq)~=sign(vel_dir))
            vel(i)=0;
        else
            vel(i)=(vel_esq+vel_dir)/2;
        end
        
    end
    vel;
   
end

function teta = parabolica(P,tm,vABC,at_max)
t = 0;

for i=0:0.1:10
    t = t + 1;
    if(i<=tm(1))
        tetai = P(1);
        vi = 0;
        tmi = 0;
        at = sign(P(2)-P(1))*at_max;
        teta(t) = tetai + vi*(i-tmi) + 0.5*at*(i-tmi)^2;
        tetaf_1 = teta(t);
    end
   
    if(tm(1)< i <= 3-tm(2)/2)
        tetai = tetaf_1;
        vi = vABC(1);   % vi = vAB
        tmi = tm(1);    % tmi = tmA
        at = 0;
        teta(t) = tetai + vi*(i-tmi) + 0.5*at*(i-tmi)^2;
        tetaf_2 = teta(t);
    end
    
    if(3-tm(2)/2 < i <= 3+tm(2)/2)
        tetai = tetaf_2;
        vi = vABC(1);
        tmi = 3-tm(2)/2;
        at = sign(vABC(2)-vABC(1))*at_max;
        teta(t) = tetai + vi*(i-tmi) + 0.5*at*(i-tmi)^2;
        tetaf_3 = teta(t);
    end
    
    if(3+tm(2)/2 < i <= 10-tm(3))
        tetai = tetaf_3;
        vi = vABC(2);   % vi = vBC
        tmi = 3+tm(2)/2; 
        at = 0;
        teta(t) = tetai + vi*(i-tmi) + 0.5*at*(i-tmi)^2;
        tetaf_4 = teta(t);
    end
    
    if(10-tm(3) < i <= 10)
        tetai = tetaf_4;
        vi = vABC(2);
        tmi = 10-tm(3);
        at = sign(P(2)-P(3))*at_max;
        teta(t) = tetai + vi*(i-tmi) + 0.5*at*(i-tmi)^2;
        tetaf_5 = teta(t);
    end
end
end


    
    
    
