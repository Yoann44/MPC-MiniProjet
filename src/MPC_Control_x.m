classdef MPC_Control_x < MPC_Control
  
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
      umax =  0.3;
      uF = [1;-1];
      uf = [umax;umax];
      
      % Limit on states (to stay in lineare part)
      amax =  0.035;
      xF = [0 1 0 0 ; 0 -1 0 0];
      xf = [amax ; amax];
      
      figure;
      sgtitle("Terminal set for x controler")
      subplot(3, 2, 1);
      Xf.projection([1 2]).plot();
      ylabel("Beta")
      xlabel("Velocity beta")
    
      subplot(3, 2, 2);
      Xf.projection([1 3]).plot();
      ylabel("Velocity x")
      xlabel("Velocity beta")
      
      subplot(3, 2, 3);
      Xf.projection([1 4]).plot();
      ylabel("x")
      xlabel("Velocity beta")
      
      subplot(3, 2, 4);
      Xf.projection([2 3]).plot();
      ylabel("Velocity x")
      xlabel("Beta")
      
      subplot(3, 2, 5);
      Xf.projection([2 4]).plot();
      ylabel("x")
      xlabel("Beta")

      subplot(3, 2, 6);
      Xf.projection([3 4]).plot();
      ylabel("x")
      xlabel("Velocity x")
      
      
      
      
      % Cost
      Q    = diag([0.1 1.0 0.1 0.1]);
      R     = 1.0;
      
      sys = LTISystem('A',mpc.A,'B',mpc.B);
      sys.x.penalty = QuadFunction(Q);
      sys.u.penalty = QuadFunction(R);
      sys.x.min(2) = -amax; sys.x.max(2) = amax;
      sys.u.min = [-umax]; sys.u.max = [umax];
      Xf = sys.LQRSet;
      Qf = sys.LQRPenalty;
      
      for k = 1:N-1
          con = con + (x(:,k+1) == mpc.A*x(:,k) + mpc.B*u(:,k));
          con = con + (xF*x(:,k) <= xf);
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
      umax =  0.3;
      uF = [1;-1];
      uf = [umax;umax];
      
      % Limit on states (to stay in lineare part)
      amax =  0.035;
      xF = [0 1 0 0 ; 0 -1 0 0];
      xf = [amax ; amax];
      
      % Constrains
      con = con + (xF*xs <= xf); % Condition on state
      con = con + (uF*us <= uf); % Condition on input
      con = con + (xs == mpc.A*xs + mpc.B*us);
      con = con + (ref == mpc.C*xs);         % Condition to satify the referance
      
      % Objective
      obj   = obj + us^2;              % Minimize the input at steady state
      
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      
      % Compute the steady-state target
      target_opt = optimizer(con, obj, sdpsettings('solver', 'gurobi'), ref, {xs, us});
      
    end
  end
end
