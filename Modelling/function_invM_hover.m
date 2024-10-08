function invM = function_invM_hover(Ixx,Iyy,Izz,L,Theta,alpha,d,mb,r)
%function_invM_hover
%    invM = function_invM_hover(Ixx,Iyy,Izz,L,Theta,ALPHA,D,MB,R)

%    This function was generated by the Symbolic Math Toolbox version 9.2.
%    22-Sep-2024 12:04:40

t2 = Theta+alpha;
t3 = L.^2;
t5 = Theta.*2.0;
t6 = alpha.*2.0;
t7 = d.^2;
t8 = mb.^2;
t11 = 1.0./mb;
t12 = 1.0./r;
t14 = Ixx.*Iyy.*2.0;
t15 = Iyy.*Izz.*2.0;
t4 = t3.^2;
t9 = cos(t5);
t10 = cos(t2);
t13 = t12.^2;
t16 = mb.*t3;
t20 = Iyy.*mb.*t7;
t24 = t2.*2.0;
t33 = t3.*t7.*t8;
t17 = Ixx.*t9;
t18 = Izz.*t9;
t19 = t10.^2;
t21 = Ixx.*t16.*2.0;
t22 = Iyy.*t16.*4.0;
t23 = Izz.*t16.*2.0;
t25 = t9.*t14;
t26 = t9.*t15;
t27 = cos(t24);
t28 = t16./2.0;
t30 = t4.*t8.*4.0;
t31 = -t20;
t38 = -t33;
t29 = -t17;
t32 = Iyy.*t17.*-2.0;
t34 = t16.*t19;
t35 = t16.*t17.*2.0;
t36 = t16.*t18.*2.0;
t37 = t27./2.0;
t41 = t16.*t27;
t43 = t27.*t28;
t48 = t22+t30+t33;
t50 = t22+t30+t38;
t39 = -t35;
t40 = -t34;
t42 = -t41;
t44 = t41.*(-1.0./2.0);
t45 = t37-1.0./2.0;
t46 = Iyy+t16+t40;
t49 = Iyy+t28+t44;
t54 = Ixx+Izz+t16+t18+t29+t42;
t56 = t45.*t48;
t57 = t45.*t50;
t47 = 1.0./t46;
t51 = 1.0./t49;
t55 = 1.0./t54;
t58 = -t56;
t59 = -t57;
t52 = L.*t10.*t12.*t47;
t60 = t14+t15+t20+t21+t23+t26+t32+t36+t39+t58;
t61 = t14+t15+t21+t23+t26+t31+t32+t36+t39+t59;
t53 = -t52;
t62 = (t11.*t13.*t51.*t55.*t60)./2.0;
t63 = (t11.*t13.*t51.*t55.*t61)./2.0;
invM = reshape([t62,t63,t53,t63,t62,t53,t53,t53,1.0./(Iyy+t16.*sin(t2).^2)],[3,3]);
