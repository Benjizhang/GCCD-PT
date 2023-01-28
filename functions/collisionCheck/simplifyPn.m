%  Takes a row vector, removes all zeros at the beginning of the vector,
%       and returns the remaining elements of the vector
%  MAKE SURE at least output 0 at the end

function vec = simplifyPn(w)    
    index = find(round(w,5) ~= 0, 1, 'first');
    if ~isempty(index)
        vec = w(index : end);
    else
        vec = 0;
    end
end