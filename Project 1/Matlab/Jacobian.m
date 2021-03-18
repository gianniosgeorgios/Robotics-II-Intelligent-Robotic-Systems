syms q1 q2 q3 q4 q5 q6 q7
syms l1 l2 l3 l4 l5

syms x y z w

%x = l4 * sin ( theta1 )
%y = l4 * cos ( theta1 )
%z = l5 * sin ( theta2 )
%w = l5 * cos ( theta2 )

% Ai is D-H Homogeneous Transformation Matrix
% from frame (i-1) to (i)

A1 = Transfer(0,0,l1,q1);
A2 = Transfer(-pi/2,0,0,q2);
A3 = Transfer(pi/2,0,l2,q3);
A4 = Transfer(pi/2,l3,0,q4); 
A5 = Transfer(pi/2,x,y,q5);
A6 = Transfer(pi/2,0,0,q6);
A7 = Transfer(-pi/2,z,w,q7);

% Creating Transfer matrices

T2= simplify(A1*A2); 
T3= simplify(T2*A3); 
T4= simplify(T3*A4); 
T5= simplify(T4*A5); 
T6= simplify(T5*A6);
T7= simplify(T6*A7);

% (pex,pey,pez) are coordinates
% of End Effector 
 
pex= T7(1,4);
pey= T7(2,4);
pez= T7(3,4);

% Jacobian Computation
J = simplify(jacobian([pex,pey,pez],[q1,q2,q3,q4,q5,q6,q7]));

% Finding gradients for criterion of subtask2 

ks3 = gradient(T3(2,4), [q1,q2])
ks4 = gradient(T4(2,4), [q1,q2,q3])




