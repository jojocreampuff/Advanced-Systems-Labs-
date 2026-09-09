function [total, tracking_cost, control_cost] = calculate_cost(error,U,dt)

    Q = eye(12);
    R = eye(4);
    
    tracking_cost = dt*sum( sum( error .* (Q*error), 1 ) );
    control_cost = dt*sum( sum( U .* (R*U), 1 ) );
    total = tracking_cost + control_cost;
    
end