classdef MPC_Control_yaw < MPC_Control
  
  methods
    % Design a YALMIP optimizer object that takes a steady-state state
    % and input (xs, us) and returns a control input
    function ctrl_opt = setup_controller(mpc)

      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % INPUTS
      %   x(:,1) - initial state (estimate)
      %   xs, us - steady-state target
      % OUTPUTS
      %   u(:,1) - input to apply to the system
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

      [n,m] = size(mpc.B);
      
      % Steady-state targets (Ignore this before Todo 3.2)
      xs = sdpvar(n, 1);
      us = sdpvar(m, 1);
      
      % SET THE HORIZON HERE
      N = 15;
      
      % Predicted state and input trajectories
      x = sdpvar(n, N);
      u = sdpvar(m, N-1);
      

      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 

      % NOTE: The matrices mpc.A, mpc.B, mpc.C and mpc.D are 
      %       the DISCRETE-TIME MODEL of your system

      % WRITE THE CONSTRAINTS AND OBJECTIVE HERE
      con = [];
      obj = 0;
      
      % Limit on inputs
      umax =  0.2;
      uF = [1;-1];
      uf = [umax;umax];
      
      % Cost
      Q    = eye(n);
      R    = 0.2 .* eye(m);
      
      sys = LTISystem('A',mpc.A,'B',mpc.B);
      sys.x.penalty = QuadFunction(Q);
      sys.u.penalty = QuadFunction(R);
      %sys.x.min = [-10;-10;-amax;-10]; sys.x.max = [10;10;amax;10];
      sys.u.min = [-umax]; sys.u.max = [umax];
      Xf = sys.LQRSet;
      Qf = sys.LQRPenalty;
      
      for k = 1:N-1
          con = con + (x(:,k+1) == mpc.A*x(:,k) + mpc.B*u(:,k));
          con = con + (uF*u(:,k) <= uf);
          obj = obj + (x(:,k)-xs)'*Q*(x(:,k)-xs);
          obj = obj + (u(:,k)-us)'*R*(u(:,k)-us);
      end
      
      %Final cost and constrain
      con = con + (Xf.A*x(:,N) <= Xf.b + Xf.A*xs);
      obj = obj + ((x(:,N)-xs)'*Qf.H*(x(:,N)-xs));
      
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      
      
      ctrl_opt = optimizer(con, obj, sdpsettings('solver','gurobi'), ...
        {x(:,1), xs, us}, u(:,1));
    end
    
    
    % Design a YALMIP optimizer object that takes a position reference
    % and returns a feasible steady-state state and input (xs, us)
    function target_opt = setup_steady_state_target(mpc)

      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % INPUTS
      %   ref    - reference to track
      % OUTPUTS
      %   xs, us - steady-state target
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

      % Steady-state targets
      n = size(mpc.A,1);
      xs = sdpvar(n, 1);
      us = sdpvar;
      
      % Reference position (Ignore this before Todo 3.2)
      ref = sdpvar;            
            
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 
      % You can use the matrices mpc.A, mpc.B, mpc.C and mpc.D
      con = [];
      obj = 0;
      
      % Limit on inputs
      umax =  0.2;
      uF = [1;-1];
      uf = [umax;umax];
      
      % Constrains
      con = con + (uF*us <= uf);    % Condition on input
      con = con + (xs == mpc.A*xs + mpc.B*us);% Condition to satify the referance
      con = con + (ref == mpc.C*xs);% Condition to satify the referance
      
      % Objective
      obj   = obj + us^2;           % Minimize the input at steady state
      
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      
      
      % Compute the steady-state target
      target_opt = optimizer(con, obj, sdpsettings('solver', 'gurobi'), ref, {xs, us});
      
    end
  end
end
