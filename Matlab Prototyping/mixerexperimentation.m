%NOTE TO SELF -> Can fully constrain the system by creating a constraint
%that F1 +F2/2 = Favg where F avg is the input thrust given by the pilot
clear all
%previous state is initial condition
x0 = [0,0,0];
a = [.1,.2,0];
out = newtons(x0,.01,.4,a)
function F = mix(conds)
    %constant design parameters (All units are metric)
    l = 1;
    h = .2;
    %inertia in all axis
    Ii = 1;
    Ik = 1;
    Ij =1;
    F1 = .5;
    a = [.1,.2,0];
    F2 = conds(1);
    theta = conds(2);
    phi = conds(3);
    F(1) = l*(F1*cos(theta)-F2*cos(phi))/Ii-a(1);
    F(2) = h*(F1*sin(theta)+F2*sin(phi))/Ij -a(2);
    F(3) = l*(F2*sin(phi)-F1*sin(theta))/Ik - a(3);
end
%calculates the acceleration based on control outputs
function a = acceleration(F2,theta,phi)
    %constant design parameters (All units are metric)
    l = 1;
    h = .2;
    %inertia in all axis
    Ii = 1;
    Ik = 1;
    Ij =1;
    F1 = .5;
    a(1) = l*(F1*cos(theta)-F2*cos(phi))/Ii;
    a(2) = h*(F1*sin(theta)+F2*sin(phi))/Ij;
    a(3) = l*(F2*sin(phi)-F1*sin(theta))/Ik;
end
%TODO add a maximum number of solutions that the algorithim has to converge
%to a solution
%stuff for custom newtons method
%x0 -> initial guess for our control outputs (should be our previous state)
% thresh -> acceptable erorr before we converge to a solution
% F1 -> desired thrust of the first motor
%a -> desired acceleration
function output = newtons(x0,thresh,F1,a)
    %system constants (m)
    l = 1;
    h = .2;
    %inertia in all axis (SI units)
    Ii = 1;
    Ik = 1;
    Ij =1;
    %goal
    ai = a(1);
    aj = a(2);
    ak = a(3);
    %iterate over our algorithim until we reach a solution starting with
    %our inital guess
    x = x0;
    error =1;
    while error >=thresh
        %xn+1 = xn - (f(xn)/f'(xn))
        x = x - (calc(x,l,h,Ii,Ij,Ik,F1,a)/derivitive(x,l,h,Ii,Ij,Ik,F1));
        error = abs(calc(x,l,h,Ii,Ij,Ik,F1,a)-0);
    end
    output = x;
end
function d = derivitive(x,l,h,Ii,Ij,Ik,F1)
    F2 = x(1);
    theta = x(2);
    phi = x(3);
    d(1) = l*(-F1*sin(theta)+F2*sin(phi))/Ii;
    d(2) = h*(F1*cos(theta)+F2*cos(phi))/Ij;
    d(3) = l*(F2*cos(phi)-F1*cos(theta))/Ik;
end
%calculates value of function with current conditions
function F = calc(x,l,h,Ii,Ij,Ik,F1,a)
    F2 = x(1);
    theta = x(2);
    phi = x(3);
    F(1) = l*(F1*cos(theta)-F2*cos(phi))/Ii-a(1);
    F(2) = h*(F1*sin(theta)+F2*sin(phi))/Ij -a(2);
    F(3) = l*(F2*sin(phi)-F1*sin(theta))/Ik - a(3);
end