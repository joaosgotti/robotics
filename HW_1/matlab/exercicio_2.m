%==========================================%
% Robótica                                 %
% Homework 1                               %
% João Vítor Sgotti Veiga (nº 2017170653)  %
% Miguel Silva (nº 2017257766)             %
%==========================================%

syms x1;
syms x2;
syms x3;
syms x4;
syms x5;

tab = [x1,0,0,pi/2,0;x2,0,4,0,pi/2;x3,0,2,0,0;x4,0,0,-pi/2,-pi/2;x5,1,0,0,0];
tam = [-pi/2,pi/2;-pi/3,pi/4;-pi/2,pi/2;-pi/2,pi/2;-pi,pi];

[T, oTi, tn, oTni] = desenha(tab, tam);

function [T, oTi, tn, oTni] = desenha(tab, tam)
    view(3);
    grid on;
    id1 = eye(4); id2 = eye(4);
    axis([-10 10 -10 10 -10 10]);
    xlabel('EIXO X');ylabel('EIXO Y');zlabel('EIXO Z');
    hold on;

    [qnt,~] = size(tab);
    

    for i=1:1:qnt        
        T(:,:,i) =  [ cos(tab(i,1)+tab(i,5)),-sin(tab(i,1)+tab(i,5))*cos(tab(i,4)),sin(tab(i,1)+tab(i,5))*sin(tab(i,4)),tab(i,3)*cos(tab(i,1)+tab(i,5));sin(tab(i,1)+tab(i,5)),cos(tab(i,1)+tab(i,5))*cos(tab(i,4)),-cos(tab(i,1)+tab(i,5))*sin(tab(i,4)),tab(i,3)*sin(tab(i,1)+tab(i,5));0,sin(tab(i,4)),cos(tab(i,4)),tab(i,2);0,0,0,1];
        
        oTi(:,:,i) = id1 * T(:,:,i);
        id1 = oTi(:,:,i);                    
        tn(:,:,i) = eval(subs(T(:,:,i), tab(i,1), 0));
        oTni(:,:,i) = id2 * tn(:,:,i);
        id2 = oTni(:,:,i);  
    end
    
    E1 = trplot(oTni(:,:,1),'arrow','width', 1,'3D', 'color' , 'b');
    E2 = trplot(oTni(:,:,2),'arrow','width', 1,'3D', 'color' , 'm');
    E3 = trplot(oTni(:,:,3),'arrow','width', 1,'3D', 'color' , 'c');
    E4 = trplot(oTni(:,:,4),'arrow','width', 1,'3D', 'color' , 'g');
    E5 = trplot(oTni(:,:,5),'arrow','width', 1,'3D', 'color' , 'b');
    
    X = [oTni(1,4,1), oTni(1,4,2), oTni(1,4,3), oTni(1,4,4), oTni(1,4,5)]; 
    Y = [oTni(2,4,1), oTni(2,4,2), oTni(2,4,3), oTni(2,4,4), oTni(2,4,5)];
    Z = [oTni(3,4,1), oTni(3,4,2), oTni(3,4,3), oTni(3,4,4), oTni(3,4,5)];
    
    HandleLine = line(X,Y,Z,'Color','r');
    
    for ag1 = tam(1,1):0.5:tam(1,2)
        for ang2 = tam(2,1):0.5:tam(2,2)
            for ang3 = tam(3,1):1:tam(3,2)
                for ang4 = tam(4,1):1.2:tam(4,2)
                    for ang5 = tam(5,1):1.3:tam(5,2)
                        ot1 = eval(subs(T(:,:,1), 'x1', ag1));
                        ot2 = ot1*eval(subs(T(:,:,2), 'x2', ang2));
                        ot3 = ot2*eval(subs(T(:,:,3), 'x3', ang3));
                        ot4 = ot3*eval(subs(T(:,:,4), 'x4', ang4));
                        ot5 = ot4*eval(subs(T(:,:,5), 'x5', ang5));
                         
                        set(E1, 'Matrix', ot1);set(E2, 'Matrix', ot2);set(E3, 'Matrix', ot3);set(E4, 'Matrix', ot4);set(E5, 'Matrix', ot5);
                        
                        X = [ot1(1,4), ot2(1,4), ot3(1,4), ot4(1,4), ot5(1,4)]; 
                        Y = [ot1(2,4), ot2(2,4), ot3(2,4), ot4(2,4), ot5(2,4)];
                        Z = [ot1(3,4), ot2(3,4), ot3(3,4), ot4(3,4), ot5(3,4)];
                        
                        set(HandleLine, 'XData', X);set(HandleLine, 'YData', Y);set(HandleLine, 'ZData', Z);
    
                        pause(0.000008);
                    end
                end
            end
        end
    end
                       
end



