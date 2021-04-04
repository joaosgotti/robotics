clear;clc;

l1=2;l3=2;

syms th1 pr2 th3;

posicao = [pi/2 3 pi/2]

PJ_DH = [th1     0      l1   pi/2    pi/2
         0       pr2    0    -pi/2   0
         th3     0      0    pi/2    0
         0       l3     0    0       pi/2];   

[Ti,Tf_generica] = MGD_HD(PJ_DH);


PJ_DH_numerica = eval(subs(PJ_DH,[th1,pr2,th3],[0 0 0]))


L = DrawRobotRPR(PJ_DH_numerica);

Robot2 = SerialLink(L,'name','Robot2');
Tf_generica = simplify(Tf_generica)

Robot2.teach();



Tf_numerica=eval(subs(Tf_generica,[th1 pr2 th3],posicao));
[t1, pr2, t3] = inversa(Tf_numerica,l1,l3);
[t1, pr2, t3]

function [t1 pr2 t3] = inversa(Tf_numerica,l1,l3)
    
    xe = Tf_numerica(1,4);
    ye = Tf_numerica(2,4);
    phi = atan2(Tf_numerica(1,2),Tf_numerica(1,3));

    
    pr2=sqrt( ( xe-l3*cos(phi) )^2 + ( ye-l3*sin(phi) )^2 - l1^2 );
    t1 = atan2(ye-l3*sin(phi),xe-l3*cos(phi)) - atan2(l1,pr2);
    t3 = phi - t1;
    
end



function L = DrawRobotRPR(tab);
L(1) = Link('revolute',      'd', double(tab(1,1)), 'a', double(tab(1,3)), 'alpha', double(tab(1,4)), 'offset', double(tab(1,5)),'qlim',[0 pi]);
L(2) = Link('prismatic'	,'theta', double(tab(2,2)), 'a', double(tab(2,3)), 'alpha', double(tab(2,4)), 'offset',double(tab(2,5)),'qlim',[0 pi]);
L(3) = Link('revolute',      'd', double(tab(3,1)), 'a', double(tab(3,3)), 'alpha', double(tab(3,4)), 'offset',double(tab(3,5)),'qlim',[0 pi]);
L(4) = Link('revolute',      'd', double(tab(4,1)), 'a', double(tab(4,4)), 'alpha', double(tab(4,4)), 'offset',double(tab(4,5)), 'qlim', [0 0]);
end