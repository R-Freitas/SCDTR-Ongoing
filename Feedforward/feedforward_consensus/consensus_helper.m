close all, clear all; clc; format shortg;

% SYSTEM / COST FUNCTION 
k11 = 2; k12 = 1; k21 = 1; k22 = 2;
L1 = 150; o1 = 30; L2 = 80; o2 = 0;
K = [k11, k12 ; k21 , k22];
L = [L1;L2]; o = [o1;o2];

c1 = 1; c2 = 1; q1 = 1; q2 = 1;
c = [c1 c2]; Q = [q1 0; 0 q2];
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ----------------------------------------------------- %
%                 SOLVE WITH CONSENSUS
% ----------------------------------------------------- %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
rho = 0.6;     
% ----------------------------------------------------- %
%node 1 initialization
d1 = [0;0];             % d[N_ELEMENTS]
d1_av = [0;0];          % d_av[N_ELEMENTS]
d2_copy = [0;0];        % d_copies[N_ELEMENTS][N_ELEMENTS]
y1 = [0;0];             
k1 = [k11;k12];         % unused
% ----------------------------------------------------- %
%node 2 initialization
d2 = [0;0];
d2_av = [0;0];
d1_copy = [0;0];
y2 = [0;0];
k2 = [k21;k22]; 
% ----------------------------------------------------- %

%iterations
iterations = 20;
for i=1:iterations,
  % node 1
   d11_best = -1; % d_best
   d12_best = -1; % d_best
   min_best_1(i) = 100000; %big number // min_best
   sol_unconstrained = 1;
   sol_boundary_linear = 1;
   sol_boundary_0 = 1;
   sol_boundary_100 = 1;
   sol_linear_0 = 1;
   sol_linear_100 = 1;
   z11 = -c1 - y1(1) + rho*d1_av(1);
   z12 = -y1(2) + rho*d1_av(2);
   u1 = o1-L1;
   u2 = 0;
   u3 = 100;
   p11 = 1/(rho+q1);
   p12 = 1/rho;
   n = k11*k11*p11 + k12*k12*p12;
   w1 = -k11*p11*z11-k12*z12*p12;
   w2 = -z11*p11;
   w3 = z11*p11;
   
   %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   %            compute unconstrained minimum
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   d11u = p11*z11;
   d12u = p12*z12;
   buffer1_u(1,i) = d11u;
   buffer1_u(2,i) = d12u;
   %check feasibility of unconstrained minimum using local constraints
   if (d11u < 0), sol_unconstrained = 0; end;
   if (d11u > 100), sol_unconstrained = 0; end;
   if (k11*d11u + k12*d12u < L1-o1), sol_unconstrained = 0; end;
   % compute function value and if best store new optimum
   buffer1_u(3,i) = 0.5*q1*d11u^2 + c1*d11u + y1(1)*(d11u-d1_av(1)) + ...
           y1(2)*(d12u-d1_av(2)) + rho/2*(d11u-d1_av(1))^2 + rho/2*(d12u-d1_av(2))^2;
   if sol_unconstrained, 
       min_unconstrained = 0.5*q1*d11u^2 + c1*d11u + y1(1)*(d11u-d1_av(1)) + ...
           y1(2)*(d12u-d1_av(2)) + rho/2*(d11u-d1_av(1))^2 + rho/2*(d12u-d1_av(2))^2;
       if min_unconstrained < min_best_1(i),
           d11_best = d11u;
           d12_best = d12u;
           min_best_1(i) = min_unconstrained;
       end;
   end;
  
   %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   %    compute minimum constrained to linear boundary   
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   d11bl = p11*z11+p11*k11/n*(w1-u1);
   d12bl = p12*z12+p12*k12/n*(w1-u1);  
   buffer1_l(1,i) = d11bl;
   buffer1_l(2,i) = d12bl;
   %check feasibility of minimum constrained to linear boundary
   if (d11bl < 0), sol_boundary_linear = 0; end;
   if (d11bl > 100), sol_boundary_linear = 0; end;
   % compute function value and if best store new optimum
   buffer1_l(3,i) = 0.5*q1*d11bl^2 + c1*d11bl + y1(1)*(d11bl-d1_av(1)) + ...
           y1(2)*(d12bl-d1_av(2)) + rho/2*(d11bl-d1_av(1))^2 + rho/2*(d12bl-d1_av(2))^2;
   if sol_boundary_linear, 
        min_boundary_linear = 0.5*q1*d11bl^2 + c1*d11bl + y1(1)*(d11bl-d1_av(1)) + ...
           y1(2)*(d12bl-d1_av(2)) + rho/2*(d11bl-d1_av(1))^2 + rho/2*(d12bl-d1_av(2))^2;
       if min_boundary_linear < min_best_1(i),
           d11_best = d11bl;
           d12_best = d12bl;
           min_best_1(i) = min_boundary_linear;
       end;
   end;
   
   %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   %compute minimum constrained to 0 boundary
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   d11b0 = 0;
   d12b0 = p12*z12;
   buffer1_b0(1,i) = d11b0;
   buffer1_b0(2,i) = d12b0;
   %check feasibility of minimum constrained to 0 boundary
   if (d11b0 > 100), sol_boundary_0 = 0; end;
   if (k11*d11b0 + k12*d12b0 < L1-o1), sol_boundary_0 = 0; end;
   % compute function value and if best store new optimum
   buffer1_b0(3,i) = 0.5*q1*d11b0^2 + c1*d11b0 + y1(1)*(d11b0-d1_av(1)) + ...
           y1(2)*(d12b0-d1_av(2)) + rho/2*(d11b0-d1_av(1))^2 + rho/2*(d12b0-d1_av(2))^2;
   if sol_boundary_0, 
        min_boundary_0 = 0.5*q1*d11b0^2 + c1*d11b0 + y1(1)*(d11b0-d1_av(1)) + ...
           y1(2)*(d12b0-d1_av(2)) + rho/2*(d11b0-d1_av(1))^2 + rho/2*(d12b0-d1_av(2))^2;
       if min_boundary_0 < min_best_1(i),
           d11_best = d11b0;
           d12_best = d12b0;
           min_best_1(i) = min_boundary_0;
       end;
   end;
   
   %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   %    compute minimum constrained to 100 boundary
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   d11b100 = 100;
   d12b100 = p12*z12;
   buffer1_b100(1,i) = d11b100;
   buffer1_b100(2,i) = d12b100;
   %check feasibility of minimum constrained to 100 boundary
   if (d11b0 < 0), sol_boundary_100 = 0; end;
   if (k11*d11b100 + k12*d12b100 < L1-o1), sol_boundary_100 = 0; end;
   % compute function value and if best store new optimum
       buffer1_b100(3,i) = 0.5*q1*d11b100^2 + c1*d11b100 + y1(1)*(d11b100-d1_av(1)) + ...
           y1(2)*(d12b100-d1_av(2)) + rho/2*(d11b100-d1_av(1))^2 + rho/2*(d12b100-d1_av(2))^2;
   if sol_boundary_100, 
        min_boundary_100 = 0.5*q1*d11b100^2 + c1*d11b100 + y1(1)*(d11b100-d1_av(1)) + ...
           y1(2)*(d12b100-d1_av(2)) + rho/2*(d11b100-d1_av(1))^2 + rho/2*(d12b100-d1_av(2))^2;
       if min_boundary_100 < min_best_1(i),
           d11_best = d11b100;
           d12_best = d12b100;
           min_best_1(i) = min_boundary_100;
       end;
   end;
   
   %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   % compute minimum constrained to linear and zero boundary
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   common = (rho+q1)/((rho+q1)*n-k11*k11);
   det1 = common;
   det2 = -k11*common;
   det3 = det2;
   det4 = n*(rho+q1)*common;
   x1 = det1*w1 + det2*w2;
   x2 = det3*w1 + det4*w2;
   v1 = det1*u1 + det2*u2; %u2 = 0 so this can be simplified
   v2 = det3*u1 + det4*u2; %u2 = 0 so this can be simplified
   d11l0 = z11/(rho+q1) + p11*k11*(x1-v1) + p11*(x2-v2);
   d12l0 = z12/rho      + p12*k12*(x1-v1);
   buffer1_l0(1,i) = d11l0;
   buffer1_l0(2,i) = d12l0;
      %check feasibility
   if (d11l0 > 100), sol_linear_0 = 0; end;
   % compute function value and if best store new optimum
   buffer1_l0(3,i) = 0.5*q1*d11l0^2 + c1*d11l0 + y1(1)*(d11l0-d1_av(1)) + ...
           y1(2)*(d12l0-d1_av(2)) + rho/2*(d11l0-d1_av(1))^2 + rho/2*(d12l0-d1_av(2))^2;
   if sol_linear_0, 
        min_linear_0 = 0.5*q1*d11l0^2 + c1*d11l0 + y1(1)*(d11l0-d1_av(1)) + ...
           y1(2)*(d12l0-d1_av(2)) + rho/2*(d11l0-d1_av(1))^2 + rho/2*(d12l0-d1_av(2))^2;
       if min_linear_0 < min_best_1(i),
           d11_best = d11l0;
           d12_best = d12l0;
           min_best_1(i) = min_linear_0;
       end;
   end;
   
   %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   % compute minimum constrained to linear and 100 boundary
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   common = (rho+q1)/((rho+q1)*n-k11*k11);
   det1 = common;
   det2 = k11*common;
   det3 = det2;
   det4 = n*(rho+q1)*common;
   x1 = det1*w1 + det2*w3;
   x2 = det3*w1 + det4*w3;
   v1 = det1*u1 + det2*u3; 
   v2 = det3*u1 + det4*u3; 
   d11l100 = p11*z11+p11*k11*(x1-v1)-p11*(x2-v2);
   d12l100 = p12*z12+p12*k12*(x1-v1);
   buffer1_l100(1,i) = d11l100;
   buffer1_l100(2,i) = d12l100;
   %check feasibility
   if (d11l100 < 0), sol_linear_100 = 0; end;
   % compute function value and if best store new optimum
   buffer1_l100(3,i) = 0.5*q1*d11l100^2 + c1*d11l100 + y1(1)*(d11l100-d1_av(1)) + ...
           y1(2)*(d12l100-d1_av(2)) + rho/2*(d11l100-d1_av(1))^2 + rho/2*(d12l100-d1_av(2))^2;         
   if sol_linear_100, 
        min_linear_100 = 0.5*q1*d11l100^2 + c1*d11l100 + y1(1)*(d11l100-d1_av(1)) + ...
           y1(2)*(d12l100-d1_av(2)) + rho/2*(d11l100-d1_av(1))^2 + rho/2*(d12l100-d1_av(2))^2;
       if min_linear_100 < min_best_1(i),
           d11_best = d11u;
           d12_best = d12u;
           min_best_1_(i) = min_linear_100;
       end;
    end;
    
   %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   %store data and save for next cycle
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   best_d11(i) = d11_best;
   best_d12(i) = d12_best;
   d1 = [d11_best;d12_best];
   buffer_d1(:,i) = d1;
   %compute average with available knowledge
   d1_av = (d1+d2_copy)/2;
   buffer_d1_av(:,i) = d1_av;
   %update local lagrangian
   y1 = y1 + rho*(d1-d1_av);
   buffer_y1(:,i) = y1;


% ----------------------------------------------------------------------- %

  % node 2 
  d21_best = -1;
   d22_best = -1;
   min_best_2(i) = 100000; %big number
   sol_unconstrained = 1;
   sol_boundary_linear = 1;
   sol_boundary_0 = 1;
   sol_boundary_100 = 1;
   sol_linear_0 = 1;
   sol_linear_100 = 1;
   z22 = -c2 - y2(2) + rho*d2_av(2);
   z21 = -y2(1) + rho*d2_av(1);
   u1 = o2-L2;
   u2 = 0;
   u3 = 100;
   p22 = 1/(rho+q2);
   p21 = 1/rho;
   n = k22*k22*p22 + k21*k21*p21;
   w1 = -k22*p22*z22-k21*z21*p21;
   w2 = -z22*p22;
   w3 = z22*p22;
   %compute unconstrained minimum
   d21u = p21*z21;
   d22u = p22*z22;
   buffer2_u(1,i) = d21u;
   buffer2_u(2,i) = d22u;
   %check feasibility of unconstrained minimum using local constraints
   if (d22u < 0), sol_unconstrained = 0; end;
   if (d22u > 100), sol_unconstrained = 0; end;
   if (k21*d21u + k22*d22u < L2-o2), sol_unconstrained = 0; end;
   % compute function value and if best store new optimum
   buffer2_u(3,i) = 0.5*q2*d22u^2 + c2*d22u + y2(1)*(d21u-d2_av(1)) + ...
           y2(2)*(d22u-d2_av(2)) + rho/2*(d21u-d2_av(1))^2 + rho/2*(d22u-d2_av(2))^2;
   if sol_unconstrained, 
        min_unconstrained = 0.5*q2*d22u^2 + c2*d22u + y2(1)*(d21u-d2_av(1)) + ...
           y2(2)*(d22u-d2_av(2)) + rho/2*(d21u-d2_av(1))^2 + rho/2*(d22u-d2_av(2))^2;
       if min_unconstrained < min_best_2(i),
           d21_best = d21u;
           d22_best = d22u;
           min_best_2(i) = min_unconstrained;
       end;
   end;
   %compute minimum constrained to linear boundary   
   d21bl = p21*z21+p21*k21/n*(w1-u1);
   d22bl = p22*z22+p22*k22/n*(w1-u1);
   buffer2_l(1,i) = d21bl;
   buffer2_l(2,i) = d22bl;   
   %check feasibility of minimum constrained to linear boundary
   if (d22bl < 0), sol_boundary_linear = 0; end;
   if (d22bl > 100), sol_boundary_linear = 0; end;
   % compute function value and if best store new optimum
   buffer2_l(3,i) = 0.5*q2*d22bl^2 + c2*d22bl + y2(1)*(d21bl-d2_av(1)) + ...
           y2(2)*(d22bl-d2_av(2)) + rho/2*(d21bl-d2_av(1))^2 + rho/2*(d22bl-d2_av(2))^2;
   if sol_boundary_linear, 
        min_boundary_linear = 0.5*q2*d22bl^2 + c2*d22bl + y2(1)*(d21bl-d2_av(1)) + ...
           y2(2)*(d22bl-d2_av(2)) + rho/2*(d21bl-d2_av(1))^2 + rho/2*(d22bl-d2_av(2))^2;
       if min_boundary_linear < min_best_2(i),
           d21_best = d21bl;
           d22_best = d22bl;
           min_best_2(i) = min_boundary_linear;
       end;
   end;
   %compute minimum constrained to 0 boundary
   d22b0 = 0;
   d21b0 = p21*z21;
   buffer2_b0(1,i) = d21b0;
   buffer2_b0(2,i) = d22b0;   
     %check feasibility of minimum constrained to 0 boundary
   if (k21*d21b0 + k22*d22b0 < L2-o2), sol_boundary_0 = 0; end;
   if (d22b0 > 100), sol_boundary_0 = 0; end;
   % compute function value and if best store new optimum
    buffer2_b0(3,i) = 0.5*q2*d22b0^2 + c2*d22b0 + y2(1)*(d21b0-d2_av(1)) + ...
           y2(2)*(d22b0-d2_av(2)) + rho/2*(d21b0-d2_av(1))^2 + rho/2*(d22b0-d2_av(2))^2;
   if sol_boundary_0, 
        min_boundary_0 = 0.5*q2*d22b0^2 + c2*d22b0 + y2(1)*(d21b0-d2_av(1)) + ...
           y2(2)*(d22b0-d2_av(2)) + rho/2*(d21b0-d2_av(1))^2 + rho/2*(d22b0-d2_av(2))^2;
       if min_boundary_0 < min_best_2(i),
           d21_best = d21b0;
           d22_best = d22b0;
           min_best_2(i) = min_boundary_0;
       end;
   end;
   %compute minimum constrained to 100 boundary
   d22b100 = 100;
   d21b100 = p21*z21;
   buffer2_b100(1,i) = d21b100;
   buffer2_b100(2,i) = d22b100;   
   %check feasibility of minimum constrained to 100 boundary
   if (k21*d21b100 + k22*d22b100 < L2-o2), sol_boundary_100 = 0; end;
   if (d22b100 < 0), sol_boundary_100 = 0; end;
   % compute function value and if best store new optimum
   buffer2_b100(3,i) = 0.5*q2*d22b100^2 + c2*d22b100 + y2(1)*(d21b100-d2_av(1)) + ...
           y2(2)*(d22b100-d2_av(2)) + rho/2*(d21b100-d2_av(1))^2 + rho/2*(d22b100-d2_av(2))^2;
   if sol_boundary_100, 
        min_boundary_100 = 0.5*q2*d22b100^2 + c2*d22b100 + y2(1)*(d21b100-d2_av(1)) + ...
           y2(2)*(d22b100-d2_av(2)) + rho/2*(d21b100-d2_av(1))^2 + rho/2*(d22b100-d2_av(2))^2;
       if min_boundary_100 < min_best_2(i),
           d21_best = d21b100;
           d22_best = d22b100;
           min_best_2(i) = min_boundary_100;
       end;
   end;
   % compute minimum constrained to linear and zero boundary
   common = (rho+q2)/((rho+q2)*n-k22*k22);
   det1 = common;
   det2 = -k22*common;
   det3 = det2;
   det4 = n*(rho+q2)*common;
   x1 = det1*w1 + det2*w2;
   x2 = det3*w1 + det4*w2;
   v1 = det1*u1 + det2*u2; %u2 = 0 so this can be simplified
   v2 = det3*u1 + det4*u2; %u2 = 0 so this can be simplified
   d22l0 = p22*z22+p22*k22*(x1-v1)+p22*(x2-v2);
   d21l0 = p21*z21+p21*k21*(x1-v1);
   buffer2_l0(1,i) = d21l0;
   buffer2_l0(2,i) = d22l0;    
   %check feasibility
   if(d22l0 > 100), sol_linear_0 = 0; end;
   % compute function value and if best store new optimum
   buffer2_l0(3,i) = 0.5*q2*d22l0^2 + c2*d22l0 + y2(1)*(d21l0-d2_av(1)) + ...
           y2(2)*(d22l0-d2_av(2)) + rho/2*(d21l0-d2_av(1))^2 + rho/2*(d22l0-d2_av(2))^2;
   if sol_linear_0, 
        min_linear_0 = 0.5*q2*d22l0^2 + c2*d22l0 + y2(1)*(d21l0-d2_av(1)) + ...
           y2(2)*(d22l0-d2_av(2)) + rho/2*(d21l0-d2_av(1))^2 + rho/2*(d22l0-d2_av(2))^2;
       if min_linear_0 < min_best_2(i),
           d21_best = d21l0;
           d22_best = d22l0;
           min_best_2(i) = min_linear_0;
       end;
   end;
   % compute minimum constrained to linear and 100 boundary
   common = (rho+q2)/((rho+q2)*n-k22*k22);
   det1 = common;
   det2 = k22*common;
   det3 = det2;
   det4 = n*(rho+q2)*common;
   x1 = det1*w1 + det2*w3;
   x2 = det3*w1 + det4*w3;
   v1 = det1*u1 + det2*u3; 
   v2 = det3*u1 + det4*u3; 
   d22l100 = p22*z22+p22*k22*(x1-v1)-p22*(x2-v2);
   d21l100 = p21*z21+p21*k21*(x1-v1);
   buffer2_l100(1,i) = d21l100;
   buffer2_l100(2,i) = d22l100;    
   %check feasibility
   if (d22l100 < 0), sol_linear_100 = 0; end;
   %now must choose the minimum among the feasible solutions
   % compute function value and if best store new optimum
   buffer2_l100(3,i) = 0.5*q2*d22l100^2 + c2*d22l100 + y2(1)*(d21l100-d2_av(1)) + ...
           y2(2)*(d22l100-d2_av(2)) + rho/2*(d21l100-d2_av(1))^2 + rho/2*(d22l100-d2_av(2))^2;
   if sol_linear_100, 
        min_linear_100 = 0.5*q2*d22l100^2 + c2*d22l100 + y2(1)*(d21l100-d2_av(1)) + ...
           y2(2)*(d22l100-d2_av(2)) + rho/2*(d21l100-d2_av(1))^2 + rho/2*(d22l100-d2_av(2))^2;
       if min_linear_100 < min_best_2(i),
           d21_best = d21u;
           d22_best = d22u;
           min_best_2(i) = min_linear100;
       end;
   end;
   

   %store data and save for next cycle
   best_d21(i) = d21_best;
   best_d22(i) = d22_best;
   d2 = [d21_best;d22_best];
   buffer_d2(:,i) = d2;
   %DEBUG: check with matlab quadprog
%    Q = [rho, 0; 0 rho+q2];
%    c = [y2(1)-rho*d2_av(1),c2+y2(2)-rho*d2_av(2)];
%    A = [-k21 -k22;0 -1; 0 1];
%    b = [o2-L2, 0, 100];
%    d2_ = quadprog(Q,c,A,b,[],[],[],[]);
   
   % Compute average with available data
   d2_av = (d1_copy+d2)/2;
   buffer_d2_av(:,i) = d2_av;
   % Update local lagrangian
   y2 = y2 + rho*(d2-d2_av);
   buffer_y2(:,i) = y2;
   
   % send node 1 solution to neighboors
   d1_copy = d1;
   % send solution to neighbors
   d2_copy = d2;
   
   d_copies = [d1_copy'; d2_copy']
   
   %save data for plots
   av1(i) = d1_av(1);
   av2(i) = d1_av(2); 
end;

%% Print buffers

% disp('-----------------------------------------------------------');
% 
% buffer1_u
% buffer1_l
% buffer1_b0
% buffer1_b100
% buffer1_l0
% buffer1_l100
% 
% buffer_d1
% buffer_d1_av
% buffer_y1
% 
% disp('-----------------------------------------------------------');
% 
% buffer2_u
% buffer2_l
% buffer2_b0
% buffer2_b100
% buffer2_l0
% buffer2_l100
% 
% buffer_d2
% buffer_d2_av
% buffer_y2


% SOLVE WITH MATLAB QUADPROG
A = -K; b = [o1-L1; o2-L2];
lb = [0;0]; ub = [100;100];
% disp('Matlab solutions')
d = quadprog(Q,c,A,b,[],[],lb,ub);
l = K*d+o;

% disp('Consensus Solutions')
% d_ = d2_av
% l_ = K*d_+o

% % PLOTS
figure(10);
plot(1:iterations, av1, 1:iterations, av2);
grid minor;
legend('d_1','d_2');
title(sprintf('Evolution of primal variables (rho = %f)', rho));
xlabel('Iteration');
ylabel('Duty cycle [%]')
set(gcf, 'Position', get(0, 'Screensize'));
% figure(15);
% t = 0:100;
% constr1 = (L1-o1)/k12-(k11/k12)*t;
% constr2 = (L2-o2)/k22-(k21/k22)*t;
% [x,y]=meshgrid(t,t);
% hold on;
% z = c1*x+c2*y+q1*x.^2+q2*y.^2;
% contour(x,y,z);
% plot(t,constr1,t,constr2,'LineWidth',2);
% plot(t,zeros(size(t)),'k','LineWidth',2);
% plot(zeros(size(t)),t,'k','LineWidth',2);
% plot(t,100*ones(size(t)),'k','LineWidth',2);
% plot(100*ones(size(t)),t,'k','LineWidth',2);
% plot(av1,av2,'--','LineWidth',2);
% plot(av1,av2,'bo');
% title('trajectory');
% xlabel('d_1');
% ylabel('d_2');
% plot(d(1),d(2),'r*')
% axis([-10,110,-10,110]);
% hold off;
