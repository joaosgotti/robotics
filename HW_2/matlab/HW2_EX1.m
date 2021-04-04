clear;clc;

l1=4;
l2=3;
l3=2;

syms th1 th2 th3;

PJ_DH = [th1 0   l1   0     0
         th2 0   l2   0     0
         th3 0   0    pi/2  pi/2
         0   l3  0    0     pi/2];

     
[Ti,Tf_generica] = MGD_HD(PJ_DH);

L = DrawRobotRRR(PJ_DH);
Robot = SerialLink(L,'name','Robot');
Tf_generica = simplify(Tf_generica)


pause();

posicao = deg2rad([ 0, 0, 0 ,0
                   10, 20,30,0
                   90, 90,90,0]) ; 

for i = 1:3
    Tf_numerica = eval(subs(Tf_generica,[th1 th2 th3 0],posicao(i,:))) 
    test_fkine = Robot.fkine(posicao(i,:))
    Robot.plot(posicao(i,:));
    [t1, t2, t3] = inversa(Tf_numerica,l1,l2,l3);
    posicao_inv = rad2deg([t1 t2 t3])
    pause();
end


function L = DrawRobotRRR(tab)

    L(1) = Link('revolute', 'd', double(tab(1,2)), 'a', double(tab(1,3)), 'alpha', double(tab(1,4)), 'offset', double(tab(1,5)),'qlim',[0 pi]);
    L(2) = Link('revolute', 'd', double(tab(2,2)), 'a', double(tab(2,3)), 'alpha', double(tab(2,4)), 'offset',double(tab(2,5)),'qlim',[0 pi]);
    L(3) = Link('revolute', 'd', double(tab(3,2)), 'a', double(tab(3,3)), 'alpha', double(tab(3,4)), 'offset',double(tab(3,5)),'qlim',[0 pi]);
    L(4) = Link('revolute', 'd', double(tab(4,2)), 'a', double(tab(4,4)), 'alpha', double(tab(4,4)), 'offset',double(tab(4,5)), 'qlim', [0 0]);
end

function [t1,t2,t3] = inversa(tab,l1,l2,l3)
    xe= tab(1,4)-l3*tab(1,3 );    
    ye= tab(2,4)-l3*tab(2,3);
    phi = atan2(tab(1,2),tab(1,3));
    
    t2 = acos((xe^2 + ye^2 - l1^2 - l2^2)/(2*l1*l2));
    t1 = atan2(ye,xe) - atan2(l2*sin(t2),l1+l2*cos(t2));
    t3= phi - t1 - t2;  
end