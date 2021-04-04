clear;clc;

l1=10;l3=15;

syms th1 pr2 th3 th4 th5 th6;

posicao = [0 0 0 0 0 0 ]

PJ_DH = [th1       0      l1   pi/2     pi/2
         0         pr2    0    -pi/2     0
         th3       0      10    pi/2     0
         th4       0      10    -pi/2    0
         th5       0      10    pi/2     0
         th6      l3      0    0        pi/2  ];  
     
     
     
[Ti,Tf_generica] = MGD_HD_6dim(PJ_DH);

PJ_DH_numerica=eval(subs(PJ_DH,[th1 pr2 th3 th4 th5 th6],posicao));

L = DrawRobotRPR(PJ_DH_numerica);

Robot2 = SerialLink(L,'name','Robot2');
Tf_generica = simplify(Tf_generica)

Robot2.teach();




function [t1 pr2 t3] = inversa(Tf_numerica,l1,l3)
    
    xe = Tf_numerica(1,4);
    ye = Tf_numerica(2,4);
    phi = atan2(Tf_numerica(1,2),Tf_numerica(1,3));

    
    pr2=sqrt( ( xe-l3*cos(phi) )^2 + ( ye-l3*sin(phi) )^2 - l1^2 );
    t1 = atan2(ye-l3*sin(phi),xe-l3*cos(phi)) - atan2(l1,pr2);
    t3 = phi - t1;
    
end



function L = DrawRobotRPR(tab);

L(1) = Link('revolute', 'd', double(tab(1,2)), 'a', double(tab(1,3)), 'alpha', double(tab(1,4)), 'offset', double(tab(1,5)),'qlim',[0 pi]);
L(2) = Link('prismatic'	,'theta', double(tab(1,2)), 'a', double(tab(2,3)), 'alpha', double(tab(2,4)), 'offset',double(tab(2,5)),'qlim',[0 15]);
L(3) = Link('revolute', 'd', double(tab(3,2)), 'a', double(tab(3,3)), 'alpha', double(tab(3,4)), 'offset',double(tab(3,5)),'qlim',[0 pi]);
L(4) = Link('revolute', 'd', double(tab(4,2)), 'a', double(tab(4,3)), 'alpha', double(tab(4,4)), 'offset',double(tab(4,5)),'qlim',[0 2*pi]);
L(5) = Link('revolute', 'd', double(tab(5,2)), 'a', double(tab(5,3)), 'alpha', double(tab(5,4)), 'offset',double(tab(5,5)),'qlim',[0 2*pi]);
L(6) = Link('revolute', 'd', double(tab(6,2)), 'a', double(tab(6,3)), 'alpha', double(tab(6,4)), 'offset',double(tab(6,5)),'qlim',[0 2*pi]);
end



