%% 4º exercício
clc, clear all;
syms d1 t2 t3;

PJ_DH_t2 = [    0        d1       0      -pi/2    25     
                t2        0       200      0       0
                t3        0       0      pi/2      0
              -pi/2      25       0        0       0  ];

% robot plot
L = DrawRobotRRR(PJ_DH_t2);
Robot = SerialLink(L,'name','PRP - Robot');
Robot.teach([50 -pi/4 pi/4 0],"view",'x');

% modelo geometrico direto do maniulador
[Ti,Tf] = MGD_HD(PJ_DH_t2);
Ti=simplify(Ti);
Tf=simplify(Tf)

Jacob = Jacobiano(Ti,Tf,d1,t2,t3)
disp("a primeira coluna das velocidades angulares é toda igual a zero por se tratar de uma junta prismatica")

function L = DrawRobotRRR(PJ_DH_t2)    
    L(1) = Link('prismatic', 'theta',double(PJ_DH_t2(1,1)),'a',double(PJ_DH_t2(1,3)),'alpha',double(PJ_DH_t2(1,4)),'offset',double(PJ_DH_t2(1,5)),'qlim',[0 200] );
    L(2) = Link('revolute',      'd',double(PJ_DH_t2(2,2)),'a',double(PJ_DH_t2(2,3)),'alpha',double(PJ_DH_t2(2,4)),'offset',double(PJ_DH_t2(2,5)),'qlim',[-pi 0] );
    L(3) = Link('revolute',      'd',double(PJ_DH_t2(3,2)),'a',double(PJ_DH_t2(3,3)),'alpha',double(PJ_DH_t2(3,4)),'offset',double(PJ_DH_t2(3,5)),'qlim',[0  pi] );
    L(4) = Link('revolute',      'd',double(PJ_DH_t2(4,2)),'a',double(PJ_DH_t2(4,3)),'alpha',double(PJ_DH_t2(4,4)),'offset',double(PJ_DH_t2(4,5)),'qlim',[0 2*pi] );
end

function Jacob = Jacobiano(Ti,Tf,d1,t2,t3)
    oPg = simplify([Tf(1,4);Tf(2,4);Tf(3,4)]);
    Jacob_v = simplify([diff(oPg,'d1') diff(oPg,'t2') diff(oPg,'t3')]);
    oRo = eye(3);
    oR1 = simplify(Ti(:,:,1));
    oR2 = simplify(Ti(:,:,1)*Ti(:,:,2));
    Jacob_w = [oRo(1:3,3) oR1(1:3,3) oR2(1:3,3)];
    Jacob = [Jacob_v;Jacob_w];
end
     