function [ctrl] = ctrlNMPC(quad)

    import casadi.*
    opti = casadi.Opti(); 
    % Optimization problem
    N = 10; % MPC horizon [SET THIS VARIABLE]
    h = 1/5;
    
    %????decision variables?????????
    X = opti.variable(12,N+1); % state trajectory variables
    U = opti.variable(4, N); % control trajectory (throttle, brake)
    
    X0 = opti.parameter(12,1);  % initial state
    REF = opti.parameter(4,1); % reference position [x,y,z,yaw]
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%% MY CODE HERE BELOW %%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    f = @(x,u) quad.f(x, u);
    
    f_discrete = @(x,u) RK4(x,u,h,f);
    
    opti.minimize(...
      2.0*(X(10,:) - REF(1))*(X(10,:) - REF(1))' + ... % Go to ref X
      2.0*(X(11,:) - REF(2))*(X(11,:) - REF(2))' + ... % Go to ref Y
      2.0*(X(12,:) - REF(3))*(X(12,:) - REF(3))' + ... % Go to ref Z
      2.0*(X(6,:) - REF(4))*(X(6,:) - REF(4))' + ... % Go to ref Yaw
      1.0*(X(quad.ind.vel(1),:))*(X(quad.ind.vel(1),:))' + ... % Minimize Speed on X
      1.0*(X(quad.ind.vel(2),:))*(X(quad.ind.vel(2),:))' + ... % Minimize Speed on Y
      1.0*(X(quad.ind.vel(3),:))*(X(quad.ind.vel(3),:))' + ... % Minimize Speed on Z
      0.1*U(1,:)*U(1,:)' + ... % Minimize cmd
      0.1*U(2,:)*U(2,:)' + ... % Minimize cmd
      0.1*U(3,:)*U(3,:)' + ... % Minimize cmd
      0.1*U(4,:)*U(4,:)');      % Minimize cmd
  
    % ---- multiple shooting --------
    for k=1:N % loop over control intervals
        opti.subject_to(X(:,k+1) == f_discrete(X(:,k), U(:,k)));
    end
  
    opti.subject_to(0 <= U <= 1);  % control is limited
    
    opti.subject_to(X(:,1)==X0);   % use initial position

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%% MY CODE HERE ABOVE %%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    ctrl = @(x,ref) evalctrl(x, ref, opti, X0, REF, X, U);
end

function u = evalctrl(x, ref, opti, X0, REF, X, U)
    %????Set the initial state and reference????
    opti.set_value(X0, x);
    opti.set_value(REF, ref);
    
    %????Setup solver NLP??????
    ops = struct('ipopt', struct('print_level',0, 'tol', 1e-3), 'print_time', false);
    opti.solver('ipopt', ops);
    
    %????Solve the optimization problem????
    sol = opti.solve();
    assert(sol.stats.success == 1, 'Error computing optimal input');
    
    u = opti.value(U(:,1));
    
    % Use the current solution to speed up the next optimization
    opti.set_initial(sol.value_variables());
    opti.set_initial(opti.lam_g, sol.value(opti.lam_g));
end

function [x_next] = RK4(X,U,h,f)
%
% Inputs : 
%    X, U current state and input
%    h    sample period
%    f    continuous time dynamics f(x,u)
% Returns
%    State h seconds in the future
%

% Runge-Kutta 4 integration
% write your function here
   k1 = f(X,        U);
   k2 = f(X+h/2*k1, U);
   k3 = f(X+h/2*k2, U);
   k4 = f(X+h*k3,   U);
   x_next = X + h/6*(k1+2*k2+2*k3+k4);
end