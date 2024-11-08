clear all
%previous state is initial condition
fun = @mix;
x0 = [.5,0,0];
[sol] = fsolve(fun,x0)
acceleration(sol(1),sol(2),sol(3))
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
