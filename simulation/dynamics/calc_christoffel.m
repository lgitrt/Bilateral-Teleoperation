function C = calc_christoffel(D, j, i, k, coordinates)
%CALC_CHRISTOFFEL Differentiate with respect to the actual model coordinates.
    C = 0.5 * (diff(D(k,j), coordinates(i)) + ...
        diff(D(k,i), coordinates(j)) - diff(D(i,j), coordinates(k)));
end
