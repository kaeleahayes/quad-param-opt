function J = cost_function(xi)
    %COST_FUNCTION Computes the cost function for parameter optimization based
    %on overall error from simulated state to state from logged flight data
    %   INPUTS:
    %       - xi: a matrix which includes the simulated states across time
    %       steps
    %
    %   OUTPUTS:
    %       - J: a scalar representing the cost associated with the current
    %       parameter values and simulation to recorded data error

    % code based on given MATLAB solution to homework problem
    N = length(xi);
    Q = diag([2, 2, 1]); % fix Q later to match appropriate state dimension
    J = 0;
    
    % fix xi/xhat indexing to match correct state dimension
    for i = 1:N
        J = J + dt*(xi(i,1:3) - xhat(i,1:3))*Q*(xi(i,1:3) - xhat(i,1:3))';
    end

end

