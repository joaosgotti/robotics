%% 1ºexercicio
clc, clear all;
syms d1 t2 d3 t4 t5;

PJ_DH =    [ pi/2   d1  0   pi/2   0   
             t2     0   0   pi/2   pi/2   
             0      d3  0   -pi/2  0
             t4     0   0   pi/2   0
             t5     5   0   0      0 ];
     
[Ti,Tf] = MGD_HD(PJ_DH);
Ti=simplify(Ti);
Tf=simplify(Tf)
Jacob = Jacobiano(Tf,Ti,d1,t2,d3,t4,t5);

L = DrawRobotRRR(PJ_DH);
Robot = SerialLink(L,'name','RPP RR - Robot');
Robot.teach([10 0 10 0 0],"workspace",[-30 30 -30 30 -30 30])

function Jacob = Jacobiano(Tf,Ti,d1,t2,d3,t4,t5)
    oPg = simplify([Tf(1,4);Tf(2,4);Tf(3,4)]);
    
    Jacob_v = simplify([diff(oPg,d1) diff(oPg,t2) diff(oPg,d3) diff(oPg,t4) diff(oPg,t5)]);
    
    oRo = eye(3);
    oR1 = simplify(Ti(:,:,1));
    oR2 = simplify(Ti(:,:,1)*Ti(:,:,2));
    oR3 = simplify(Ti(:,:,1)*Ti(:,:,2)*Ti(:,:,3));
    oR4 = simplify(Ti(:,:,1)*Ti(:,:,2)*Ti(:,:,3)*Ti(:,:,4));
    oR5 = simplify(Ti(:,:,1)*Ti(:,:,2)*Ti(:,:,3)*Ti(:,:,4)*Ti(:,:,5));
    
    Jacob_w = [oRo(1:3,3) oR1(1:3,3) oR2(1:3,3) oR3(1:3,3) oR4(1:3,3) ];
    
    Jacob = [Jacob_v;Jacob_w]
    
end

function L = DrawRobotRRR(PJ_DH)    
    L(1) = Link('prismatic', 'theta',double(PJ_DH(1,1)),'a',double(PJ_DH(1,3)),'alpha',double(PJ_DH(1,4)),'offset',double(PJ_DH(1,5)),'qlim',[0 20] );  
    L(2) = Link('revolute',      'd',double(PJ_DH(2,2)),'a',double(PJ_DH(2,3)),'alpha',double(PJ_DH(2,4)),'offset',double(PJ_DH(2,5)),'qlim',[0 2*pi] );
    L(3) = Link('prismatic', 'theta',double(PJ_DH(3,1)),'a',double(PJ_DH(3,3)),'alpha',double(PJ_DH(3,4)),'offset',double(PJ_DH(3,5)),'qlim',[0 20] );  
    L(4) = Link('revolute',      'd',double(PJ_DH(4,2)),'a',double(PJ_DH(4,3)),'alpha',double(PJ_DH(4,4)),'offset',double(PJ_DH(4,5)),'qlim',[0 2*pi] );
    L(5) = Link('revolute',      'd',double(PJ_DH(5,2)),'a',double(PJ_DH(5,3)),'alpha',double(PJ_DH(5,4)),'offset',double(PJ_DH(5,5)),'qlim',[0 2*pi] );   
end