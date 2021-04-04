%% Exercício 12
clc, clear all;
syms t1 d2 d3 t4 t5;

PJ_DH =    [    t1        0       0        -pi/2   0     
                0         d2      0        pi/2    0
                0         d3      0        -pi/4   0
                t4        1       0        pi/2    -pi/2  
                t5        0       0        pi/2    0
                -pi/2     1       0        0       0];

% robot plot
L = DrawRobotRRR(PJ_DH);
Robot = SerialLink(L,'name','RPP-RR');
%Robot.teach([pi 1  1 pi pi 0],"workspace",[-5 5 -5 5 -5 5]);

% modelo geometrico direto do maniulador
[Ti,Tf] = MGD_HD(PJ_DH);
Ti=simplify(Ti);
Tf=simplify(Tf);

Jacob = Jacobiano(Ti,Tf,t1,d2,d3,t4,t5)

function L = DrawRobotRRR(PJ_D)    
    L(1) = Link('revolute',      'd',double(PJ_D(1,2)),'a',double(PJ_D(1,3)),'alpha',double(PJ_D(1,4)),'offset',double(PJ_D(1,5)),'qlim',[0 2*pi] );
    L(2) = Link('prismatic', 'theta',double(PJ_D(2,1)),'a',double(PJ_D(2,3)),'alpha',double(PJ_D(2,4)),'offset',double(PJ_D(2,5)),'qlim',[0 2] );
    L(3) = Link('prismatic', 'theta',double(PJ_D(3,1)),'a',double(PJ_D(3,3)),'alpha',double(PJ_D(3,4)),'offset',double(PJ_D(2,5)),'qlim',[0 2] );
    L(4) = Link('revolute',      'd',double(PJ_D(4,2)),'a',double(PJ_D(4,3)),'alpha',double(PJ_D(4,4)),'offset',double(PJ_D(4,5)),'qlim',[0 2*pi] );
    L(5) = Link('revolute',      'd',double(PJ_D(5,2)),'a',double(PJ_D(5,3)),'alpha',double(PJ_D(5,4)),'offset',double(PJ_D(5,5)),'qlim',[0 2*pi] );
    L(6) = Link('revolute',      'd',double(PJ_D(6,2)),'a',double(PJ_D(6,3)),'alpha',double(PJ_D(6,4)),'offset',double(PJ_D(6,5)),'qlim',[0 2*pi] );   
end

function Jacob = Jacobiano(Ti,Tf,t1,d2,d3,t4,t5)
    oT4 = Ti(:,:,1)*Ti(:,:,2)*Ti(:,:,3)*Ti(:,:,4);
    oP4 = simplify([oT4(1,4);oT4(2,4);oT4(3,4)]);
    Jacob_v = simplify([diff(oP4,'t1'),diff(oP4,'d2'),diff(oP4,'d3'),diff(oP4,'t4'),diff(oP4,'t5')]);
    
    oRo = eye(3);
    oR1 = simplify(Ti(:,:,1));
    oR2 = simplify(Ti(:,:,1)*Ti(:,:,2));
    oR3 = simplify(Ti(:,:,1)*Ti(:,:,2)*Ti(:,:,3));
    oR4 = simplify(Ti(:,:,1)*Ti(:,:,2)*Ti(:,:,3)*Ti(:,:,4));
    Jacob_w = simplify([oRo(1:3,3) oR1(1:3,3) oR2(1:3,3) oR3(1:3,3) oR4(1:3,3)]);
    
    Jacob = [Jacob_v;Jacob_w]
end
     