clc;
clear all;
format long;
lenPA10=[0.45 0.5 0.28 0.2];

qa=[0;-pi/4;0;pi/2;0;-pi/4;0];
qb=[0;0;0;0;0;0;0];
qc=[0;+pi/4;0;pi/2;0;-pi/4;0];
qd=[0;-pi/4;0;2*pi/3;0;-pi/4;0];
qe=[0;+pi/4;0;2*pi/3;0;-pi/4;0];
qf=[0;-pi/4;0;pi/2;0;-pi/4;0];
qg=[0;-pi/4;0;pi/2;0;-pi/4;0];
q0=[qa;pi/4;0];%may need adjust

q0=(rand(9,1)-0.5*ones(9,1))*10;
q0(9,1)=0;
%Js=JacobN(q0,lenPA10);
%errJ=Js-JacobON(q0,lenPA10,0);

[Ppx,Ppy,Ppz,PpTr]=positionN(q0,lenPA10);
pos_initial=[Ppx Ppy Ppz]
j1px=Ppx(1);
j2px=Ppx(2);
   j3px=Ppx(3);
   j4px=Ppx(4);
   j5px=Ppx(5);
   j6px=Ppx(6);
   j7px=Ppx(7);
   j8px=Ppx(8);
   j9px=Ppx(9);
   %-----------------
   j1py=Ppy(1);
   j2py=Ppy(2);
   j3py=Ppy(3);
   j4py=Ppy(4);
   j5py=Ppy(5);
   j6py=Ppy(6);
   j7py=Ppy(7);
   j8py=Ppy(8);
   j9py=Ppy(9);
   %-----------------
   j1pz=Ppz(1);
   j2pz=Ppz(2);
   j3pz=Ppz(3);
   j4pz=Ppz(4);
   j5pz=Ppz(5);
   j6pz=Ppz(6);
   j7pz=Ppz(7);
   j8pz=Ppz(8);
   j9pz=Ppz(9);
   linksX=[j1px;j2px;j3px;j4px;j5px;j6px;j7px;j8px;j9px];
   linksY=[j1py;j2py;j3py;j4py;j5py;j6py;j7py;j8py;j9py];
   linksZ=[j1pz;j2pz;j3pz;j4pz;j5pz;j6pz;j7pz;j8pz;j9pz];
%   lengthL=(j8px-j7px)^2+(j8py-j7py)^2+(j8pz-j7pz)^2;
 lengthL=(j9px-j8px)^2+(j9py-j8py)^2+(j9pz-j8pz)^2;
   lengthL=lengthL^0.5
   plot3(linksX,linksY,linksZ);