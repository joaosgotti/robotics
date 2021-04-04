function [Ti,TF] = MGD_HD(tab)
dims=size(tab);
dim=dims(1);
Ti = sym(zeros(4,4,dim));
TF=eye(4);
for i = 1:dim
    Ti(:,:,i) = [cos(tab(i,1) + tab(i,5)),-sin(tab(i,1) + tab(i,5))*cos(tab(i,4)),  sin(tab(i,1) + tab(i,5))*sin(tab(i,4)),tab(i,3)*cos(tab(i,1) + tab(i,5));
                sin(tab(i,1) + tab(i,5)), cos(tab(i,1) + tab(i,5))*cos(tab(i,4)), -cos(tab(i,1) + tab(i,5))*sin(tab(i,4)),tab(i,3)*sin(tab(i,1) + tab(i,5));
                0 , sin(tab(i,4)), cos(tab(i,4)), tab(i,2);
                0 , 0, 0 , 1];
   
    TF=TF*Ti(:,:,i);
end